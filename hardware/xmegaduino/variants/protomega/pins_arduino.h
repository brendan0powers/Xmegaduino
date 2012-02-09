/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define PROTOMEGA_TIMING // Use protmega mode for millis etc.
#define PROTOMEGA_ADC

#define REPEAT8(x) x, x, x, x, x, x, x, x
#define BV0TO7 _BV(0), _BV(1), _BV(2), _BV(3), _BV(4), _BV(5), _BV(6), _BV(7)
#define BV7TO0 _BV(7), _BV(6), _BV(5), _BV(4), _BV(3), _BV(2), _BV(1), _BV(0)

#define NUM_DIGITAL_PINS            26
#define NUM_ANALOG_INPUTS           9
#define EXTERNAL_NUM_INTERRUPTS     26

#define analogInputToDigitalPin(p)  ((p < 9) ? (p) + 17 : -1)
#define digitalPinHasPWM(p)         (((p) >= 1 && (p) <= 8) || ((p) >= 11 && (p)<= 16))

const static uint8_t SS    = 12;
const static uint8_t MOSI  = 11;
const static uint8_t MISO  = 10;
const static uint8_t SCK   = 9;

const static uint8_t SDA = 4;
const static uint8_t SCL = 3;
const static uint8_t LED_BUILTIN = 13;

const static uint8_t A0 = 17;
const static uint8_t A1 = 18;
const static uint8_t A2 = 18;
const static uint8_t A3 = 20;
const static uint8_t A4 = 21;
const static uint8_t A5 = 22;
const static uint8_t A6 = 23;
const static uint8_t A7 = 24;
const static uint8_t A8 = 25;

#define Wire xmWireE
#define Wire1 xmWireC

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 13) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) - 14)))

#ifdef ARDUINO_MAIN

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &PORTA,
        (uint16_t) &PORTB,
        (uint16_t) &PORTC,
        (uint16_t) &PORTD,
        (uint16_t) &PORTE,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST
	NOT_A_PIN,
	PE,	// D1
	PE,
	PE,
	PE,
	PD,
	PD,
	PD,
	PD,	// D8
	PC,
	PC,
	PC,
	PC,
	PC,
	PC,
	PC,
	PC,	// D16
	PB,	// A1
	PB,
	PB,
	PB,
	PA,
	PA,
	PA,
	PA,
	PA,	// A9	
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PINSLIST
	0,
        _BV(3),     // D1
        _BV(2),
        _BV(1),
        _BV(0),
        _BV(3),
        _BV(2),
        _BV(1),
        _BV(0),     // D8
        _BV(7),
        _BV(6),
        _BV(5),
        _BV(4),
        _BV(3),
        _BV(2),
        _BV(1),
        _BV(0),     // D16
        _BV(3),     // A1
        _BV(2),
        _BV(1),
        _BV(0),
        _BV(4),
        _BV(3),
        _BV(2),
        _BV(1),
        _BV(0),     // A9
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,
        TIMER_E0D,     // D1
        TIMER_E0C,
        TIMER_E0B,
       	TIMER_E0A,
        TIMER_D0D,
        TIMER_D0C,
        TIMER_D0B,
       	TIMER_D0A,     // D8
        TIMER_C2HD,
        TIMER_C2HC,
        TIMER_C2HB,
        TIMER_C2HA,
        TIMER_C2LD,
        TIMER_C2LC,
        TIMER_C2LB,
        TIMER_C2LA,     // D16
        NOT_ON_TIMER,     // A1
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,     // A9
};


const uint8_t PROGMEM digital_pin_to_dac_PGM[] = {
	// PORTLIST
	NOT_A_DAC,
	NOT_A_DAC,	// D1
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC, // D8 
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,	// D16
	1,	// A1
	0,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,
	NOT_A_DAC,	// A9	
};

const TC0_t* PROGMEM timer_to_tc0_PGM[] = {
	NULL,

	&TCC0,
	&TCC0,
	&TCC0,
	&TCC0,
	NULL,
	NULL,

	&TCD0,
	&TCD0,
	&TCD0,
	&TCD0,
	NULL,
	NULL,

	&TCE0,
	&TCE0,
	&TCE0,
	&TCE0,
	NULL,
	NULL,

	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,

	NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
};

const TC1_t* PROGMEM timer_to_tc1_PGM[] = {
	NULL,

	NULL,
	NULL,
	NULL,
	NULL,
	&TCC1,
	&TCC1,

	NULL,
	NULL,
	NULL,
	NULL,
	&TCD1,
	&TCD1,

        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,

        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,

	NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
};

const TC2_t* PROGMEM timer_to_tc2_PGM[] = {
        NULL,

        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,

        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,

	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,

        &TCC2,   
        &TCC2,
        &TCC2,
        &TCC2, 
        &TCC2,
        &TCC2,
        &TCC2,
        &TCC2,

	&TCD2,
        &TCD2,
        &TCD2,
        &TCD2,
        &TCD2,
        &TCD2,
        &TCD2,
        &TCD2,
};

const uint8_t PROGMEM timer_to_channel_PGM[] = {
	NOT_ON_TIMER,

    0,
    1,
    2,
    3,
    0,
    1,

    0,
    1,
    2,
    3,
    0,
    1,

    0,
    1,
    2,
    3,
    0,
    1,

    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,

    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
};

const uint8_t PROGMEM adc_to_channel_PGM[] = {
    11,
    10,
    9,
    8,
    4,
    3,
    2,
    1,
    0
};

#endif

#endif
