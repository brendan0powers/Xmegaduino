/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <stdio.h>
#include <Arduino.h>
#include <avr/pgmspace.h>

#define SPI_CLOCK_DIV4 SPI_PRESCALER_DIV4_gc
#define SPI_CLOCK_DIV16 SPI_PRESCALER_DIV16_gc
#define SPI_CLOCK_DIV64 SPI_PRESCALER_DI64_gc
#define SPI_CLOCK_DIV128 SPI_PRESCALER_DIV128_gc
#define SPI_CLOCK_DIV2 SPI_PRESCALER_DIV4_gc | SPI_CLK2X_bm 
#define SPI_CLOCK_DIV8 SPI_PRESCALER_DIV16_gc | SPI_CLK2X_bm 
#define SPI_CLOCK_DIV32 SPI_PRESCALER_DIV64_gc | SPI_CLK2X_bm 
//#define SPI_CLOCK_DIV64 0x07

#define SPI_MODE0 SPI_MODE_0_gc
#define SPI_MODE1 SPI_MODE_1_gc
#define SPI_MODE2 SPI_MODE_2_gc
#define SPI_MODE3 SPI_MODE_3_gc

class SPIClass {
public:
  inline static byte transfer(byte _data);

  // SPI Configuration methods

  inline static void attachInterrupt();
  inline static void detachInterrupt(); // Default

  static void begin(); // Default
  static void end();

  static void setBitOrder(uint8_t);
  static void setDataMode(uint8_t);
  static void setClockDivider(uint8_t);
};

extern SPIClass SPI;

byte SPIClass::transfer(byte _data) {
  SPIC.DATA = _data;
  while (!(SPIC.STATUS & SPI_IF_bm))
    ;
  return SPIC.DATA;
}

void SPIClass::attachInterrupt() {
  SPIC.INTCTRL = SPI_INTLVL_LO_gc;
}

void SPIClass::detachInterrupt() {
  SPIC.INTCTRL = 0;
}

#endif
