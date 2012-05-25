
#include "io.h"
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;


#define USB_PID_LEONARDO 0x0034
#define USB_PID_MICRO 0x0035
#define USB_VID 0x2341	// arduino LLC vid
#define USB_PID ARDUINO_MODEL_PID	// passed in by Makefile - 0x0034 for Leonardo, 0x0035 for MIcro

#define USB_SERIAL_STRING	'0','0','0','0','0','0','0','0','1','7','0','1'

#define OEM_NAME		'l','e','o','n','a','r','d','o'					// 8 chars

#define LED0			PORTC.OUTCLR = (1<<3)
#define LED1			PORTC.OUTSET = (1<<3)
#define TXLED0			PORTC.OUTCLR = (1<<3)
#define TXLED1			PORTC.OUTSET = (1<<3)
#define RXLED0			PORTC.OUTCLR = (1<<3)
#define RXLED1			PORTC.OUTSET = (1<<3)

#define TRANSFER_PGM		0x80
#define TRANSFER_RELEASE	0x40
#define TRANSFER_ZERO		0x20

void Transfer(u8 ep, const u8* data, int len);
void Recv(u8 ep, u8* dst, u8 len);
void Program(u8 ep, u16 page, u8 count);

#include "USBCore.h"
#include "USBDesc.h"
#include "sp_driver.h"


