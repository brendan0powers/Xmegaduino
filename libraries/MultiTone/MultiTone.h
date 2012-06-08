/*
  MultiTone.h - a library for playing multiple, high precision tones at once
  on the xmega chip.  This should work with any xmega arduino-compatible board.
  
  Unlike Tone(), MultiTone() can only work on a few, specific, "multi-tone compatible" pins.  For the protomega board, these pins are D1, D5, D11, and D13.  While MultiTone is active, Tone() will be disabled.  In addition, analogWrite won't work on pins D1-D8.  No other arduino functions should be disrupted, though.  MultiTone() does not use interrupts, so it is unaffected by interrupt timing.  

  For other boards, you'll have to look up in the documentation which board pin corresponds to which hardware pin.  The hardware pins used for MultiTone are TCE0 OC0D, TCD0 OC0D, TCC1 OC1B, and TCC0 OC0D.
 */

#ifndef __MultiTone_H__
#define __MultiTone_H__

#include <Arduino.h>

/*
 * Start a tone at the specified frequency on the specified pin.
 * Each pin can only have one tone, but that tone is independent of the
 * other pins.
 * As mentioned above, this will stop the Tone() function from working.
 */
void MultiTone(uint8_t pin, uint16_t freq);

/* 
 * Turn the multitone off at a specific pin.  This is to maintain API parity with Tone().
 */
void noMultiTone(uint8_t pin);

#endif
