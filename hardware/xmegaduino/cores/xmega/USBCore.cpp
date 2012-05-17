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
#include "stddef.h"


#ifdef USB

#ifdef USB_LED
  #define TX_RX_LED_PULSE_MS 100
  volatile uint8_t TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
  volatile uint8_t RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */
#endif

//==================================================================
//==================================================================

extern const uint16_t STRING_LANGUAGE[] PROGMEM;
extern const uint16_t STRING_IPRODUCT[] PROGMEM;
extern const uint16_t STRING_IMANUFACTURER[] PROGMEM;
extern const DeviceDescriptor USB_DeviceDescriptor PROGMEM;
extern const DeviceDescriptor USB_DeviceDescriptorA PROGMEM;

const uint16_t STRING_LANGUAGE[2] = {
	(3<<8) | (2+2),
	0x0409	// English
};

const uint16_t STRING_IPRODUCT[17] = {
	(3<<8) | (2+2*16),
#if USB_PID == USB_PID_LEONARDO	
	'A','r','d','u','i','n','o',' ','L','e','o','n','a','r','d','o'
#elif USB_PID == USB_PID_MICRO
	'A','r','d','u','i','n','o',' ','M','i','c','r','o',' ',' ',' '
#else
	'A','r','d','u','i','n','o',' ','S','o','m','m','a','t',' ',' '
#endif
};

const uint16_t STRING_IMANUFACTURER[12] = {
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
	D_DEVICE(0x00,0x00,0x00,USB_EPSIZE,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);

const DeviceDescriptor USB_DeviceDescriptorA =
	D_DEVICE(DEVICE_CLASS,0x00,0x00,USB_EPSIZE,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);

//==================================================================
//==================================================================

typedef struct USB_EP_pair{
	USB_EP_t out;
	USB_EP_t in;
} __attribute__ ((packed)) USB_EP_pair_t;

USB_EP_pair_t endpoints[USB_NUM_EP + 1] __attribute__((aligned(2), section(".usbendpoints")));

struct USB_EP_Buffer_Pair
{
  uint8_t out[USB_EPSIZE];
  uint8_t in[USB_EPSIZE];
};

// this is where we store the data for each endpoint.
USB_EP_Buffer_Pair ep_bufs[USB_NUM_EP];

enum InOrOut
{
  kIn,
  kOut
};

struct USB_EP_Data_Ptr
{
  uint8_t out;
  uint8_t in;
};
// this is where we store how much of the data we've already read
USB_EP_Data_Ptr ep_data_ptr[USB_NUM_EP];
USB_EP_Data_Ptr ep_last_stalled[USB_NUM_EP];


volatile uint8_t _usbConfiguration = 0;

static inline bool Stalled(uint8_t ep, InOrOut inOut)
{
  if(inOut == kIn)
    return endpoints[ep].in.STATUS & USB_EP_STALLF_bm;
  else
    return endpoints[ep].out.STATUS & USB_EP_STALLF_bm;
}

static inline bool TransactionComplete(uint8_t endpoint, InOrOut inOut)
{
  if(inOut == kIn)
    return ep_last_stalled[endpoint].in || endpoints[endpoint].in.STATUS & (USB_EP_TRNCOMPL0_bm | USB_EP_OVF_bm);
  else
    return ep_last_stalled[endpoint].out || endpoints[endpoint].out.STATUS & USB_EP_TRNCOMPL0_bm;
}

static inline void WaitForTransactionComplete(uint8_t endpoint, InOrOut inOut)
{
  while (!TransactionComplete(endpoint, inOut));
}

static inline InOrOut WaitForEitherInOrOut(uint8_t endpoint)
{
  while(true)
  {
    if(!(endpoints[endpoint].out.STATUS & USB_EP_TRNCOMPL0_bm))
      return kOut;
    if(!(endpoints[endpoint].in.STATUS & USB_EP_TRNCOMPL0_bm))
      return kIn;
  }
}

static inline bool WasSetupReceived(uint8_t ep)
{
  // setups are marked on both the in and out registers,
  // so it doesn't matter which one we grab.
  return endpoints[ep].out.STATUS & USB_EP_SETUP_bm;
}

static inline void ClearSetupReceived(uint8_t ep)
{
  endpoints[ep].out.STATUS &= ~USB_EP_SETUP_bm;
  ep_data_ptr[ep].out = 0;
}

// this stalls the next transaction.
static inline void Stall(uint8_t ep, InOrOut inOut)
{
  if(inOut == kIn)
  {
    ep_data_ptr[ep].in = 0;
    endpoints[ep].in.CTRL |= USB_EP_STALL_bm;
    endpoints[ep].in.STATUS = USB_EP_STALLF_bm | USB_EP_OVF_bm;
    ep_last_stalled[ep].in = true;
  }
  else
  {
    ep_data_ptr[ep].out = 0;
    endpoints[ep].out.CTRL |= USB_EP_STALL_bm;
    endpoints[ep].out.STATUS = USB_EP_STALLF_bm | USB_EP_OVF_bm;
    ep_last_stalled[ep].out = true;
  }
}

// This has several effects:
//   - clears the BUSNACK flag - this means that we'll accept the next
//     packet we receive and therefore our data will be overwritten.
//     Don't call this unless you're done with all the data in the packet.
//   - clears the TRNCOMP flag - this means that WaitForTransactionComplete
//     will wait for the *next* transaction.
// This is therefore a very important call, because packets will be 
// dropped until you call this.
static inline void ReadyForNextPacket(uint8_t ep, InOrOut inOut)
{
  if(inOut == kOut)
  {
    ep_data_ptr[ep].out = 0;
    ep_last_stalled[ep].in = false;
    endpoints[ep].out.STATUS &= ~(USB_EP_STALL_bm | USB_EP_TRNCOMPL1_bm | USB_EP_TRNCOMPL0_bm | USB_EP_BUSNACK0_bm | USB_EP_BUSNACK1_bm | USB_EP_OVF_bm);
  }
  else
  {
    ep_last_stalled[ep].in = false;
    endpoints[ep].in.STATUS &= ~(USB_EP_STALL_bm | USB_EP_TRNCOMPL1_bm | USB_EP_TRNCOMPL0_bm | USB_EP_BUSNACK0_bm |  USB_EP_BUSNACK1_bm | USB_EP_OVF_bm);
  }
}

void InitControlEP()
{
	/* Configure Control Endpoint */
	endpoints[0].out.STATUS = 0;
	endpoints[0].out.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EPSIZE_gc;
	endpoints[0].out.DATAPTR = (unsigned) ep_bufs[0].out;
	endpoints[0].in.STATUS = 0;
	endpoints[0].in.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EPSIZE_gc;
	endpoints[0].in.DATAPTR = (unsigned) ep_bufs[0].in;
    ep_last_stalled[0].in = true;
    ep_last_stalled[0].in = true;
    ep_data_ptr[0].in = ep_data_ptr[0].out = 0;
}

#define F_USB 48000000

void USB_Init()
{
	_usbConfiguration = 0;
	
	/* Configure USB clock */

    // disable PLL.
	OSC.CTRL &= ~(OSC_PLLEN_bm);
	
    // use the 2MHz oscillator, multiplied by 24 = 48MHz.
    OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | 24;
    
    // enable the 2MHz.
    OSC.CTRL |= OSC_RC2MEN_bm;

    // wait for 2MHz stability
    while(not (OSC.STATUS & OSC_RC2MRDY_bm));

    // enable PLL
    OSC.CTRL |= OSC_PLLEN_bm;

    // wait for PLL stability.
    while(not (OSC.STATUS & OSC_PLLRDY_bm));
  
	//USB clock enabled, high speed mode, uses the 32Mhz DFLL configured above
	CLK.USBCTRL = ((((F_USB / 48000000) - 1) << CLK_USBPSDIV_gp) | CLK_USBSRC_PLL_gc | CLK_USBSEN_bm);
    	
	//Reset the USB address
	USB.ADDR = 0;
	USB.EPPTR = (unsigned) &endpoints; //Set the endpoint address pointer
    
    InitControlEP();
    
	// Enable USB, Full speed mode, with USB_NUM_EP endpoints
	USB.CTRLA = USB_ENABLE_bm | USB_SPEED_bm | (USB_NUM_EP - 1);
    USB.CTRLB = USB_ATTACH_bm;
    // we need the following interrupts:
    // - setup transaction complete
    // - Start of frame
    // - Reset (encapsulated by BUSEVIE)
    USB.INTCTRLA = 0; // USB_SOFIE_bm |  USB_INTLVL_LO_gc; // low priority.
    USB.INTCTRLB = 0; // USB_SETUPIE_bm;
}


// this is a low level function that assumes that we are *sure*
// that there is at least "count" bytes left in the buffer
// for the associated input.  don't call this unless you're totally
// sure and you've already checked.
void Recv(uint8_t ep, volatile uint8_t* outBuf, uint8_t count)
{
  while (count--)
    *outBuf++ = ep_bufs[ep].out[ep_data_ptr[ep].out++];

#ifdef USB_LED
	RXLED1;					// light the RX LED
	RxLEDPulse = TX_RX_LED_PULSE_MS;	
#endif
}

static inline uint8_t Recv8(uint8_t ep)
{
  uint8_t out;
  Recv(ep, &out, 1);
  return out;
}

// send zero-size packet.
static inline void Send0(uint8_t ep)
{
  endpoints[ep].in.CNT = 0;
}

// Again, this is a dangerous private call.
// Do not call this if you aren't sure that you're not overflowing
// the buffer.
static inline void Send8(uint8_t ep, uint8_t d)
{
  ep_bufs[ep].in[ep_data_ptr[ep].in++] = d;
  
  // mark that there's another byte in the buffer.
  endpoints[ep].in.CNT = ep_data_ptr[ep].in;
}

// this is the number of bytes that remain on an endpoint.
static inline uint8_t BytesLeft(uint8_t ep, InOrOut inOut)
{
  // If the transaction isn't complete, there's nothing to read.
  if(!TransactionComplete(ep, inOut))
    return 0;

  // we have to subtract off the data pointer.
  uint16_t totalCount;
  if(inOut == kOut)
  {
    totalCount = endpoints[ep].out.CNT;
    totalCount -= ep_data_ptr[ep].out;
  }
  else
  {
    totalCount = USB_EPSIZE;
    totalCount -= ep_data_ptr[ep].in;
  }
  return totalCount;
}

//==================================================================
//==================================================================

uint8_t USBGetConfiguration(void)
{
	return _usbConfiguration;
}

#define USB_RECV_TIMEOUT

//	Number of bytes, assumes a rx endpoint
uint8_t USB_Available(uint8_t ep)
{
  return BytesLeft(ep, kOut);
}

//	Non Blocking receive
//	Return number of bytes read
int USB_Recv(uint8_t ep, void* d, int len)
{
	if (!_usbConfiguration || len < 0)
		return -1;
	
	uint8_t n = BytesLeft(ep, kOut);
	len = min(n,len);
	n = len;
	uint8_t* dst = (uint8_t*)d;
	while (n--)
		*dst++ = Recv8(ep);
	if (len && (BytesLeft(ep, kOut) == 0))	// if there's no bytes left, we're done.
      ReadyForNextPacket(ep, kOut);
	
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
  return USB_EPSIZE - BytesLeft(ep, kIn);
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
			if (ep & TRANSFER_ZERO)
			{
				while (n--)
                  Send8(ep, 0);
			}
			else if (ep & TRANSFER_PGM)
			{
				while (n--)
                  Send8(ep, pgm_read_byte(data++));
			}
			else
			{
				while (n--)
                  Send8(ep, *data++);
			}
			if ((len == 0) && (ep & TRANSFER_RELEASE))	// Release full buffer
              ReadyForNextPacket(ep, kIn);
		}
	}

#ifdef USB_LED
	TXLED1;					// light the TX LED
	TxLEDPulse = TX_RX_LED_PULSE_MS;
#endif

	return r;
}



extern const uint8_t _initEndpoints[] PROGMEM;
const uint8_t _initEndpoints[] = 
{
    0, // endpoint 0 is special and not set-up here.
	
#ifdef CDC_ENABLED
	0,		    // CDC_ENDPOINT_ACM
	1,			// CDC_ENDPOINT_OUT
	0,			// CDC_ENDPOINT_IN
#endif

#ifdef HID_ENABLED
	0		// HID_ENDPOINT_INT
#endif
};

static
void InitEP(uint8_t ep, InOrOut type)
{
  if(type == kIn)
  {
    endpoints[ep].in.STATUS = 0;
    endpoints[ep].in.CTRL = USB_EP_TYPE_BULK_gc | USB_EPSIZE_gc;
    endpoints[ep].in.DATAPTR = (unsigned) ep_bufs[ep].in;
    endpoints[ep].in.CNT = USB_EPSIZE;
    ep_data_ptr[ep].in = 0;
    ep_last_stalled[ep].in = true;
  }
  else if(type == kOut)
  {
    endpoints[ep].out.STATUS = 0;
    endpoints[ep].out.CTRL = USB_EP_TYPE_BULK_gc | USB_EPSIZE_gc;
    endpoints[ep].out.DATAPTR = (unsigned) ep_bufs[ep].out;
    ep_data_ptr[ep].out = 0;
    ep_last_stalled[ep].out = true;
  }
}

static
void InitEndpoints()
{
	for (uint8_t i = 1; i < sizeof(_initEndpoints); i++)
	{
      InitEP(i, pgm_read_byte(_initEndpoints+i) ? kOut : kIn);
	}
}

//	Handle CLASS_INTERFACE requests
static
bool ClassInterfaceRequest(Setup& setup)
{
	uint8_t i = setup.wIndex;
#ifdef CDC_ENABLED
	if (CDC_ACM_INTERFACE == i)
    {
		return CDC_Setup(setup);
    }

#endif

#ifdef HID_ENABLED
	if (HID_INTERFACE == i)
    {
		return HID_Setup(setup);
    }
#endif
	return false;
}

// this starts the sending of a new packet.
static void 
InitSend(uint8_t ep)
{
  // double check the previous transaction is done
  WaitForTransactionComplete(ep, kIn);
  ep_data_ptr[ep].in = 0;
  endpoints[ep].in.CNT = 0;
}

// This clips the sending of control data.
// it seems kind of heavy-handed.
static int _cmark = 0;
static int _cend = 0;
void InitControl(int end)
{
	_cmark = 0;
	_cend = end;
    InitSend(0);
}

// this is a blocking function that sends the current packet
// on the given endpoint.
static void
SendCurrentPacket(uint8_t ep)
{
  ReadyForNextPacket(ep, kIn);
  WaitForTransactionComplete(ep, kIn);
}

static void
AcknowledgeSetupPacket(InOrOut inout)
{
  // to acknowledge a setup transaction, we
  // do an operation in the *opposite* direction
  // with a blank packet.
  if(inout == kIn)
  {
    // receive a blank packet.
    ReadyForNextPacket(0, kOut);
    WaitForTransactionComplete(0, kOut);
  }
  else
  {
    // send a blank packet.
    InitSend(0);
    SendCurrentPacket(0);
  }
}

// send control always works on endpoint 0.
// returns true if we sent it, 
// returns false if we received something while trying to send.
static
bool SendControl(uint8_t d)
{
	if (_cmark < _cend)
	{
      Send8(0, d);
      // if we hit the end of the buffer, send the packet.
	  if (ep_data_ptr[0].in >= USB_EPSIZE)
      {
        SendCurrentPacket(0);
        InitSend(0);
      }
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
  // ensure that we're ready for the next packet.
  // this will discard any data outside of "len".
  ReadyForNextPacket(0, kOut);

  WaitForTransactionComplete(0, kOut);

  Recv(0, (uint8_t*)d,len);

  ReadyForNextPacket(0, kOut);

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
    {
      return SendConfiguration(setup.wLength);
    }

	InitControl(setup.wLength);

#ifdef HID_ENABLED
	if (HID_REPORT_DESCRIPTOR_TYPE == t)
      {
		return HID_GetDescriptor(t);
      }
#endif

	uint8_t desc_length = 0;
	const uint8_t* desc_addr = 0;
	if (USB_DEVICE_DESCRIPTOR_TYPE == t)
	{
		if (setup.wLength >= 8)
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

// this interrupt is fired when a transaction is complete.
ISR(USB_TRNCOMPL_vect)
{
}

// actually sends out the stuff in the in buffer
void USB_Flush(uint8_t ep)
{
  SendCurrentPacket(ep);
  InitSend(ep);
}

uint16_t LastFrameNumber()
{
  // frame number count is after all of the endpoint buffers.
  return *(uint16_t*)(&endpoints[USB_NUM_EP]);
}

// this is called on bus events.  we only care about start-of-frame and
// reset.
ISR(USB_BUSEVENT_vect)
{
  uint8_t intFlags = USB.INTFLAGSASET;
  USB.INTFLAGSACLR = 0xFF; // clear all interrupt flags.

  //	End of Reset
  if (intFlags & USB_RSTIF_bm)
  {
    InitControlEP();
	_usbConfiguration = 0;			// not configured yet
  }

  //	Start of Frame - happens every millisecond so we use it for TX and RX LED one-shot timing, too
  if(intFlags & USB_SOFIF_bm)
  {
#ifdef CDC_ENABLED
	USB_Flush(CDC_TX);				// Send a tx frame if found
#endif

#ifdef USB_LED		
    // check whether the one-shot period has elapsed.  if so, turn off the LED
	if (TxLEDPulse && !(--TxLEDPulse))
		TXLED0;
	if (RxLEDPulse && !(--RxLEDPulse))
		RXLED0;
#endif
}
}

//	VBUS or counting frames
//	Any frame counting?
bool USBConnected()
{
    uint16_t f = LastFrameNumber();
	delay(3);
	return f != LastFrameNumber();
}

//=======================================================================
//=======================================================================

USB_ USBConfig;

USB_::USB_()
{
}

void USB_::attach()
{
  USB_Init();
  #ifdef USB_LED
	TX_RX_LED_INIT;
  #endif
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
  // we only need to do something here if 
  // we got a setup packet on EP 0.
  if(!WasSetupReceived(0))
    return;

  // grab the setup data.
  Setup setup;
  Recv(0, (uint8_t*)&setup,8);

  // clear the flag for next time.
  ClearSetupReceived(0);

  uint8_t requestType = setup.bmRequestType;
  if (requestType & REQUEST_DEVICETOHOST)
  {
    InitSend(0);
  }

  bool ok = true;
  if (REQUEST_STANDARD == (requestType & REQUEST_TYPE))
  {
    //	Standard Requests
    uint8_t r = setup.bRequest;
    if (GET_STATUS == r)
    {
      if(requestType & REQUEST_DEVICETOHOST)
      {
        Send8(0, 0);		// TODO
        Send8(0, 0);
      }
      else
        Stall(0, kOut);
    }
    else if (CLEAR_FEATURE == r) {}
	else if (SET_FEATURE == r) {}
	else if (SET_ADDRESS == r)
    {
      // we have to ACK from old address.
      AcknowledgeSetupPacket(kOut);
      USB.ADDR = setup.wValueL;
      return;
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
      Send8(0, 1);
	}
    else if (SET_CONFIGURATION == r)
    {
      if (REQUEST_DEVICE == (requestType & REQUEST_RECIPIENT))
      {
        InitEndpoints();
        _usbConfiguration = setup.wValueL;
        ok = true;
      } else
        ok = false;
    }
    else if (GET_INTERFACE == r) {}
    else if (SET_INTERFACE == r) {}
  }
  else 
  {
    if(requestType & REQUEST_DEVICETOHOST)
      InitControl(setup.wLength);		//	Max length of transfer
    ok = ClassInterfaceRequest(setup);
  }

  if(requestType & REQUEST_DEVICETOHOST)
  {
    if (ok)
    {
      SendCurrentPacket(0);
      AcknowledgeSetupPacket(kIn);
    }
    else
    {
      Stall(0, kIn);
    }
  }
  else
  {
    if (ok)
    {
      AcknowledgeSetupPacket(kOut);
    }
  }
}

#endif 
