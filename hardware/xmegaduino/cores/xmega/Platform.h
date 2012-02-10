#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Arduino.h"

#if defined(USB)
	#include "USBDesc.h"
	#include "USBCore.h"
	#include "USBAPI.h"
#endif 

#endif
