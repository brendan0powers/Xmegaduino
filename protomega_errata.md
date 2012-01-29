# Documentation errata for the Protomega port of Arduino

## Basic timing: `delay()`, `millis()`, `micros()`

These should all basically work the same as on the regular Arduino.  

### Notes for advanced users:

The protomega uses a 16 bit timer, "TCD1" for these timing functions.  Millis are counted by a low priority interrupt, so if you disable interrupts or create a lot of higher priority interrupts, `delay()` won't work and `millis()` won't increment.  This is similar to behavior of the ATMega Arduino.  Like the ATMega Arduino, `delay()` is implemented in terms of `millis()` so it won't be incredible accurate for low millisecond counts -- e.g.  `delay(1)` could return instantly if you call right before a timer interrupt.

## Precision timing: `delayMicroseconds()`

This should be more accurate than the regular Arduino.  On the regular Arduino, you can pass in up to 16384 microseconds.  On the protomega, you can delay up to 65535 microseconds.

### Notes for advanced users:

If the chip is running at 32MHz, 16MHz, or 8MHz (32MHz is the default), the protomega will use a clock-cycle-counting busy waiting strategy.  This should be as accuate as the crystal, given that interrupts are disabled.  `delayMicroseconds()` itself will not disable interrupts for you, so if you want the best possible timing, you should call `cli()` before calling `delayMicroseconds()`.  

`delayMicroseconds` takes into account the time it takes to call the function, and the time it takes to load the 16 bit argument into registers.  The reason for this is so that `delayMicroseconds(15); delayMicroseconds(15);` will be exactly equivalent to `delayMicroseconds(30);`.

At 8MHz clock speeds, `delayMicroseconds()` is only accurate if the argument is greater than 1 and less than 32767.  At 32MHz and 16MHz all arguments should be accurate.  At clock speeds other than 32MHz, 16MHz, and 8MHz, accuracy is not guarenteed.
