/* Copyright (c) 2010, Peter Barrett  
**  
** Permission to use, copy, modify, and/or distribute this software for  
** any purpose with or without fee is hereby granted, provided that the  
** above copyright notice and this permission notice appear in all copies.  
** 
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL  
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED  
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR  
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES  
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,  
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,  
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS  
** SOFTWARE.  
*/

#include "Platform.h"
#include "USBAPI.h"
#include "USBDesc.h"

#ifdef USB

#define EP_TYPE_CONTROL				0x00
#define EP_TYPE_BULK_IN				0x81
#define EP_TYPE_BULK_OUT			0x80
#define EP_TYPE_INTERRUPT_IN		0xC1
#define EP_TYPE_INTERRUPT_OUT		0xC0
#define EP_TYPE_ISOCHRONOUS_IN		0x41
#define EP_TYPE_ISOCHRONOUS_OUT		0x40

/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
#define TX_RX_LED_PULSE_MS 100
volatile uint8_t TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
volatile uint8_t RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */

//==================================================================
//==================================================================

extern const u16 STRING_LANGUAGE[] PROGMEM;
extern const u16 STRING_IPRODUCT[] PROGMEM;
extern const u16 STRING_IMANUFACTURER[] PROGMEM;
extern const DeviceDescriptor USB_DeviceDescriptor PROGMEM;
extern const DeviceDescriptor USB_DeviceDescriptorA PROGMEM;

const u16 STRING_LANGUAGE[2] = {
	(3<<8) | (2+2),
	0x0409	// English
};

const u16 STRING_IPRODUCT[17] = {
	(3<<8) | (2+2*16),
#if USB_PID == USB_PID_LEONARDO	
	'A','r','d','u','i','n','o',' ','L','e','o','n','a','r','d','o'
#elif USB_PID == USB_PID_MICRO
	'A','r','d','u','i','n','o',' ','M','i','c','r','o',' ',' ',' '
#else
	'A','r','d','u','i','n','o',' ','S','o','m','m','a','t',' ',' '
#endif
};

const u16 STRING_IMANUFACTURER[12] = {
	(3<<8) | (2+2*11),
	'A','r','d','u','i','n','o',' ','L','L','C'
};

#ifdef CDC_ENABLED
#define DEVICE_CLASS 0x02
#else
#define DEVICE_CLASS 0x00
#endif

//	DEVICE DESCRIPTOR
const DeviceDescriptor USB_DeviceDescriptor =
	D_DEVICE(0x00,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);

const DeviceDescriptor USB_DeviceDescriptorA =
	D_DEVICE(DEVICE_CLASS,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);

//==================================================================
//==================================================================

typedef struct USB_EP_pair{
	USB_EP_t out;
	USB_EP_t in;
} __attribute__ ((packed)) USB_EP_pair_t;

USB_EP_pair_t endpoints[USB_MAXEP+1] __attribute__((aligned(2), section(".usbendpoints")));

// this is where we store the data for each endpoint.
uint8_t ep_bufs[USB_MAXEP * 2 * USB_EPSIZE];

// 

volatile uint8_t _usbConfiguration = 0;

static inline void WaitIN(uint8_t endpoint)
{
	while (!(endpoints[endpoint].in.STATUS & USB_EP_TRNCOMPL0_bm));
}

static inline void ClearIN(uint8_t endpoint)
{
  // to clear this flag you write a 1 to it.  see 20.14.1 in XMEGAAU manual
  endpoints[endpoint].in.STATUS = USB_EP_TRNCOMPL0_bm;
}

static inline void WaitOUT(uint8_t endpoint)
{
	while (!(endpoints[endpoint].out.STATUS & USB_EP_TRNCOMPL0_bm));
}

static inline void ClearOUT(uint8_t endpoint)
{
  // to clear this flag you write a 1 to it.  see 20.14.1 in XMEGAAU manual
  endpoints[endpoint].out.STATUS = USB_EP_TRNCOMPL0_bm;
}

enum InOrOut
{
  kIn,
  kOut
};

static inline InOrOut WaitForINOrOUT(uint8_t endpoint)
{
  while(true)
  {
    if(!(endpoints[endpoint].out.STATUS & USB_EP_TRNCOMPL0_bm))
      return kOut;
    if(!(endpoints[endpoint].in.STATUS & USB_EP_TRNCOMPL0_bm))
      return kIn;
  }
}

void USB_Init(void)
{
	_timeout = 0;
	_usbConfiguration = 0;
	_ejected = 0;
	
	/* Configure USB clock */
	OSC.DFLLCTRL = OSC_RC32MCREF_USBSOF_gc; //Use internal 32khz osc. to calibrate the USB clock
	//Set calibration bytes for the usb clock
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	DFLLRC32M.CALB = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBRCOSC));
	//Set the frequency of the USB osc. to 48MHz
	DFLLRC32M.COMP1 = 0x1B; //Xmega AU manual, p41
	DFLLRC32M.COMP2 = 0xB7;
	DFLLRC32M.CTRL = DFLL_ENABLE_bm; //Enable USB clock
	
	//Load USB calibration bytes
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	USB.CAL0 = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBCAL0));
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	USB.CAL1 = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBCAL1));
	
	//USB clock enabled, high speed mode, uses the 32Mhz DFLL configured above
	CLK.USBCTRL = ((((F_USB / 48000000) - 1) << CLK_USBPSDIV_gp) | CLK_USBSRC_RC32M_gc | CLK_USBSEN_bm);
	
	//Reset the USB address
	USB.ADDR = 0;
	USB.EPPTR = (unsigned) &endpoints; //Set the endpoint address pointer
	
	/* Configure Control Endpoint */
	endpoints[0].out.STATUS = 0;
	endpoints[0].out.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_size_to_gc(USB_EP0SIZE);
	endpoints[0].out.DATAPTR = (unsigned) &ep0_buf_out;
	endpoints[0].in.STATUS = USB_EP_BUSNACK0_bm;
	endpoints[0].in.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_size_to_gc(USB_EP0SIZE);
	endpoints[0].in.DATAPTR = (unsigned) &ep0_buf_in;
	
	// Enable USB, Full speed mode, with USB_MAXEP endpoints
	USB.CTRLA = USB_ENABLE_bm | USB_SPEED_bm | USB_MAXEP;
}


void Recv(volatile uint8_t* data, uint8_t count)
{
	while (count--)
		*data++ = UEDATX;
	
	RXLED1;					// light the RX LED
	RxLEDPulse = TX_RX_LED_PULSE_MS;	
}

static inline uint8_t Recv8()
{
	RXLED1;					// light the RX LED
	RxLEDPulse = TX_RX_LED_PULSE_MS;

	return UEDATX;	
}

static inline void Send8(uint8_t d)
{
	UEDATX = d;
}

static inline uint8_t FifoByteCount()
{
	return UEBCLX;
}

static inline uint8_t ReceivedSetupInt()
{
	return UEINTX & (1<<RXSTPI);
}

static inline void ClearSetupInt()
{
	UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
}

static inline void Stall()
{
	UECONX = (1<<STALLRQ) | (1<<EPEN);
}

static inline uint8_t ReadWriteAllowed()
{
	return UEINTX & (1<<RWAL);
}

static inline uint8_t Stalled()
{
	return UEINTX & (1<<STALLEDI);
}

static inline uint8_t FifoFree()
{
	return UEINTX & (1<<FIFOCON);
}

static inline void ReleaseRX()
{
	UEINTX = 0x6B;	// FIFOCON=0 NAKINI=1 RWAL=1 NAKOUTI=0 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=1
}

static inline void ReleaseTX()
{
	UEINTX = 0x3A;	// FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
}

static inline uint8_t FrameNumber()
{
	return UDFNUML;
}

//==================================================================
//==================================================================

uint8_t USBGetConfiguration(void)
{
	return _usbConfiguration;
}

#define USB_RECV_TIMEOUT
class LockEP
{
	uint8_t _sreg;
public:
	LockEP(uint8_t ep) : _sreg(SREG)
	{
		cli();
		SetEP(ep & 7);
	}
	~LockEP()
	{
		SREG = _sreg;
	}
};

//	Number of bytes, assumes a rx endpoint
uint8_t USB_Available(uint8_t ep)
{
	LockEP lock(ep);
	return FifoByteCount();
}

//	Non Blocking receive
//	Return number of bytes read
int USB_Recv(uint8_t ep, void* d, int len)
{
	if (!_usbConfiguration || len < 0)
		return -1;
	
	LockEP lock(ep);
	uint8_t n = FifoByteCount();
	len = min(n,len);
	n = len;
	uint8_t* dst = (uint8_t*)d;
	while (n--)
		*dst++ = Recv8();
	if (len && !FifoByteCount())	// release empty buffer
		ReleaseRX();
	
	return len;
}

//	Recv 1 byte if ready
int USB_Recv(uint8_t ep)
{
	uint8_t c;
	if (USB_Recv(ep,&c,1) != 1)
		return -1;
	return c;
}

//	Space in send EP
uint8_t USB_SendSpace(uint8_t ep)
{
	LockEP lock(ep);
	if (!ReadWriteAllowed())
		return 0;
	return 64 - FifoByteCount();
}

//	Blocking Send of data to an endpoint
int USB_Send(uint8_t ep, const void* d, int len)
{
	if (!_usbConfiguration)
		return -1;

	int r = len;
	const uint8_t* data = (const uint8_t*)d;
	uint8_t zero = ep & TRANSFER_ZERO;
	uint8_t timeout = 250;		// 250ms timeout on send? TODO
	while (len)
	{
		uint8_t n = USB_SendSpace(ep);
		if (n == 0)
		{
			if (!(--timeout))
				return -1;
			delay(1);
			continue;
		}

		if (n > len)
			n = len;
		len -= n;
		{
			LockEP lock(ep);
			if (ep & TRANSFER_ZERO)
			{
				while (n--)
					Send8(0);
			}
			else if (ep & TRANSFER_PGM)
			{
				while (n--)
					Send8(pgm_read_byte(data++));
			}
			else
			{
				while (n--)
					Send8(*data++);
			}
			if (!ReadWriteAllowed() || ((len == 0) && (ep & TRANSFER_RELEASE)))	// Release full buffer
				ReleaseTX();
		}
	}
	TXLED1;					// light the TX LED
	TxLEDPulse = TX_RX_LED_PULSE_MS;
	return r;
}

extern const uint8_t _initEndpoints[] PROGMEM;
const uint8_t _initEndpoints[] = 
{
	0,
	
#ifdef CDC_ENABLED
	EP_TYPE_INTERRUPT_IN,		// CDC_ENDPOINT_ACM
	EP_TYPE_BULK_OUT,			// CDC_ENDPOINT_OUT
	EP_TYPE_BULK_IN,			// CDC_ENDPOINT_IN
#endif

#ifdef HID_ENABLED
	EP_TYPE_INTERRUPT_IN		// HID_ENDPOINT_INT
#endif
};

#define EP_SINGLE_64 0x32	// EP0
#define EP_DOUBLE_64 0x36	// Other endpoints

static
void InitEP(uint8_t index, uint8_t type, uint8_t size)
{
	UENUM = index;
	UECONX = 1;
	UECFG0X = type;
	UECFG1X = size;
}

static
void InitEndpoints()
{
	for (uint8_t i = 1; i < sizeof(_initEndpoints); i++)
	{
		UENUM = i;
		UECONX = 1;
		UECFG0X = pgm_read_byte(_initEndpoints+i);
		UECFG1X = EP_DOUBLE_64;
	}
	UERST = 0x7E;	// And reset them
	UERST = 0;
}

//	Handle CLASS_INTERFACE requests
static
bool ClassInterfaceRequest(Setup& setup)
{
	uint8_t i = setup.wIndex;

#ifdef CDC_ENABLED
	if (CDC_ACM_INTERFACE == i)
		return CDC_Setup(setup);
#endif

#ifdef HID_ENABLED
	if (HID_INTERFACE == i)
		return HID_Setup(setup);
#endif
	return false;
}

int _cmark;
int _cend;
void InitControl(int end)
{
	SetEP(0);
	_cmark = 0;
	_cend = end;
}

static
bool SendControl(uint8_t d)
{
	if (_cmark < _cend)
	{
		if (!WaitForINOrOUT())
			return false;
		Send8(d);
		if (!((_cmark + 1) & 0x3F))
			ClearIN(0);	// Fifo is full, release this packet
	}
	_cmark++;
	return true;
};

//	Clipped by _cmark/_cend
int USB_SendControl(uint8_t flags, const void* d, int len)
{
	int sent = len;
	const uint8_t* data = (const uint8_t*)d;
	bool pgm = flags & TRANSFER_PGM;
	while (len--)
	{
		uint8_t c = pgm ? pgm_read_byte(data++) : *data++;
		if (!SendControl(c))
			return -1;
	}
	return sent;
}

//	Does not timeout or cross fifo boundaries
//	Will only work for transfers <= 64 bytes
//	TODO
int USB_RecvControl(void* d, int len)
{
	WaitOUT();
	Recv((uint8_t*)d,len);
	ClearOUT();
	return len;
}

int SendInterfaces()
{
	int total = 0;
	uint8_t interfaces = 0;

#ifdef CDC_ENABLED
	total = CDC_GetInterface(&interfaces);
#endif

#ifdef HID_ENABLED
	total += HID_GetInterface(&interfaces);
#endif

	return interfaces;
}

//	Construct a dynamic configuration descriptor
//	This really needs dynamic endpoint allocation etc
//	TODO
static
bool SendConfiguration(int maxlen)
{
	//	Count and measure interfaces
	InitControl(0);	
	int interfaces = SendInterfaces();
	ConfigDescriptor config = D_CONFIG(_cmark + sizeof(ConfigDescriptor),interfaces);

	//	Now send them
	InitControl(maxlen);
	USB_SendControl(0,&config,sizeof(ConfigDescriptor));
	SendInterfaces();
	return true;
}

uint8_t _cdcComposite = 0;

static
bool SendDescriptor(Setup& setup)
{
	uint8_t t = setup.wValueH;
	if (USB_CONFIGURATION_DESCRIPTOR_TYPE == t)
		return SendConfiguration(setup.wLength);

	InitControl(setup.wLength);
#ifdef HID_ENABLED
	if (HID_REPORT_DESCRIPTOR_TYPE == t)
		return HID_GetDescriptor(t);
#endif

	uint8_t desc_length = 0;
	const uint8_t* desc_addr = 0;
	if (USB_DEVICE_DESCRIPTOR_TYPE == t)
	{
		if (setup.wLength == 8)
			_cdcComposite = 1;
		desc_addr = _cdcComposite ?  (const uint8_t*)&USB_DeviceDescriptorA : (const uint8_t*)&USB_DeviceDescriptor;
	}
	else if (USB_STRING_DESCRIPTOR_TYPE == t)
	{
		if (setup.wValueL == 0)
			desc_addr = (const uint8_t*)&STRING_LANGUAGE;
		else if (setup.wValueL == IPRODUCT) 
			desc_addr = (const uint8_t*)&STRING_IPRODUCT;
		else if (setup.wValueL == IMANUFACTURER)
			desc_addr = (const uint8_t*)&STRING_IMANUFACTURER;
		else
			return false;
	}

	if (desc_addr == 0)
		return false;
	if (desc_length == 0)
		desc_length = pgm_read_byte(desc_addr);

	USB_SendControl(TRANSFER_PGM,desc_addr,desc_length);
	return true;
}

//	Endpoint 0 interrupt
ISR(USB_COM_vect)
{
    SetEP(0);
	if (!ReceivedSetupInt())
		return;

	Setup setup;
	Recv((uint8_t*)&setup,8);
	ClearSetupInt();

	uint8_t requestType = setup.bmRequestType;
	if (requestType & REQUEST_DEVICETOHOST)
		WaitIN(0);
	else
		ClearIN(0);

    bool ok = true;
	if (REQUEST_STANDARD == (requestType & REQUEST_TYPE))
	{
		//	Standard Requests
		uint8_t r = setup.bRequest;
		if (GET_STATUS == r)
		{
			Send8(0);		// TODO
			Send8(0);
		}
		else if (CLEAR_FEATURE == r)
		{
		}
		else if (SET_FEATURE == r)
		{
		}
		else if (SET_ADDRESS == r)
		{
			WaitIN(0);
			UDADDR = setup.wValueL | (1<<ADDEN);
		}
		else if (GET_DESCRIPTOR == r)
		{
			ok = SendDescriptor(setup);
		}
		else if (SET_DESCRIPTOR == r)
		{
			ok = false;
		}
		else if (GET_CONFIGURATION == r)
		{
			Send8(1);
		}
		else if (SET_CONFIGURATION == r)
		{
			if (REQUEST_DEVICE == (requestType & REQUEST_RECIPIENT))
			{
				InitEndpoints();
				_usbConfiguration = setup.wValueL;
			} else
				ok = false;
		}
		else if (GET_INTERFACE == r)
		{
		}
		else if (SET_INTERFACE == r)
		{
		}
	}
	else
	{
		InitControl(setup.wLength);		//	Max length of transfer
		ok = ClassInterfaceRequest(setup);
	}

	if (ok)
		ClearIN(0);
	else
	{
		Stall();
	}
}

void USB_Flush(uint8_t ep)
{
	SetEP(ep);
	if (FifoByteCount())
		ReleaseTX();
}

//	General interrupt
ISR(USB_GEN_vect)
{
	uint8_t udint = UDINT;
	UDINT = 0;

	//	End of Reset
	if (udint & (1<<EORSTI))
	{
		InitEP(0,EP_TYPE_CONTROL,EP_SINGLE_64);	// init ep0
		_usbConfiguration = 0;			// not configured yet
		UEIENX = 1 << RXSTPE;			// Enable interrupts for ep0
	}

	//	Start of Frame - happens every millisecond so we use it for TX and RX LED one-shot timing, too
	if (udint & (1<<SOFI))
	{
#ifdef CDC_ENABLED
		USB_Flush(CDC_TX);				// Send a tx frame if found
#endif
		
		// check whether the one-shot period has elapsed.  if so, turn off the LED
		if (TxLEDPulse && !(--TxLEDPulse))
			TXLED0;
		if (RxLEDPulse && !(--RxLEDPulse))
			RXLED0;
	}
}

//	VBUS or counting frames
//	Any frame counting?
uint8_t USBConnected()
{
	uint8_t f = UDFNUML;
	delay(3);
	return f != UDFNUML;
}

//=======================================================================
//=======================================================================

USB_ USB;

USB_::USB_()
{
}

void USB_::attach()
{
	_usbConfiguration = 0;
	UHWCON = 0x01;						// power internal reg
	USBCON = (1<<USBE)|(1<<FRZCLK);		// clock frozen, usb enabled
	PLLCSR = 0x12;						// Need 16 MHz xtal
	while (!(PLLCSR & (1<<PLOCK)))		// wait for lock pll
		;
	USBCON = ((1<<USBE)|(1<<OTGPADE));	// start USB clock
	UDIEN = (1<<EORSTE)|(1<<SOFE);		// Enable interrupts for EOR (End of Reset) and SOF (start of frame)
	UDCON = 0;							// enable attach resistor
	
	TX_RX_LED_INIT;
}

void USB_::detach()
{
}

//	Check for interrupts
//	TODO: VBUS detection
bool USB_::configured()
{
	return _usbConfiguration;
}

void USB_::poll()
{
}

#endif 
