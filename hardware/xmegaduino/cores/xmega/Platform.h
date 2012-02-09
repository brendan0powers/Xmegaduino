#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#include "Arduino.h"

#if defined(USB)
	#include "USBDesc.h"
	#include "USBCore.h"
	#include "USBAPI.h"
#endif 

#endif
