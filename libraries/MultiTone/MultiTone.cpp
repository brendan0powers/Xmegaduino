#include "MultiTone.h"
#include "pins_arduino.h"

#define timerToChannelRegister(T) ( (volatile uint16_t *)( pgm_read_word( timer_to_channel_register_PGM + (T))) )

// helper functions
namespace
{


  // edit all these together, please!
  uint32_t prescales_available[] = {1, 2, 4, 8, 64, 256, 1024};
  uint8_t  prescales_shift_vs_previous[] = {0, 1, 1, 1, 3, 2, 2};
  uint8_t prescale_ctrla[] = {TC_CLKSEL_DIV1_gc, TC_CLKSEL_DIV2_gc,
                              TC_CLKSEL_DIV4_gc, TC_CLKSEL_DIV8_gc,
                              TC_CLKSEL_DIV64_gc, TC_CLKSEL_DIV256_gc,
                              TC_CLKSEL_DIV1024_gc};
  
  uint8_t num_prescales = sizeof(prescales_available) / sizeof(uint32_t);

  uint32_t rounded_div_by_2(uint32_t in)
  {
    if(in & 1)
      in = (in >> 1) + 1;
    else
      in = (in >> 1);
    return in;
  }
  
  uint32_t rounded_shift(uint32_t in, uint8_t shift)
  {
    if(shift == 0)
      return in;

    in = in >> (shift - 1);
    return rounded_div_by_2(in);
  }

  bool GetPeriodAndPrescaleForFreq(uint16_t& period, uint8_t& prescale, uint16_t frequency)
  {
    // let's loop through the pre-scalers and try to find the best match.
    // round it.
    uint32_t required_period = (F_CPU * 2 / frequency);
    required_period = rounded_div_by_2(required_period);

    // go through from fastest to slowest, since we want the most accurate possible.
    for(unsigned int i = 0; i < num_prescales; ++i)
    {
      required_period = rounded_shift(required_period, prescales_shift_vs_previous[i]);

      if(required_period <= 0x10000) // can we do this in 16 bits?
      {
        prescale = prescale_ctrla[i];
        period = required_period - 1;
        return true;
      }
    }
    // if none of the prescalers worked, the frequency was too low! wow!
    return false;
  }

  void EnableDFLL()
  {
    // if it's already enabled, don't do anything.
    if(DFLLRC32M.CTRL & DFLL_ENABLE_bm)
      return;

    // enable the 32KHz
    OSC.CTRL |= OSC_RC32KEN_bm;

    // wait for stability
    while(not (OSC.STATUS & OSC_RC32KRDY_bm));

    // enable DFLL for extra power drain and accuracy.
    OSC.DFLLCTRL = (OSC.DFLLCTRL & ~OSC_RC32MCREF_gm) | OSC_RC32MCREF_RC32K_gc;
    DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
    
    // wait for 32MHz stability.
    while(not (OSC.STATUS & OSC_RC32MRDY_bm));
  }



  struct Timer_Pin_Map
  {
    uint8_t portIndex;
    uint8_t pinMask;
    uint8_t timerIndex;
  };

  // since the protomega uses the alternate TCC2 for pwm, and there's no defines for the timer roles of the pins in atmel's include file, we'll just hard code it by port.
  // allowed timers are: TIMER_E0D, TIMER_D0D, TIMER_C0D, TIMER_C1B.
  Timer_Pin_Map pin_to_timer_map[] = {
    {PE, 1<<3, TIMER_E0D},
    {PD, 1<<3, TIMER_D0D},
    {PC, 1<<3, TIMER_C0D},
    {PC, 1<<5, TIMER_C1B}
  };

  uint8_t pin_to_timer_map_length = sizeof(pin_to_timer_map) / sizeof(Timer_Pin_Map);

  uint8_t GetTimerForPin(uint8_t pin)
  {
    
  	uint8_t pinMask = digitalPinToBitMask(pin);
	uint8_t portIndex = digitalPinToPort(pin);

    // if it's not a pin, it's not a timer.
    if(portIndex == NOT_A_PIN)
      return NOT_ON_TIMER;

    // check each allowed timer.
    for(int i = 0; i < pin_to_timer_map_length; ++i)
    {
      if((pinMask == pin_to_timer_map[i].pinMask) and
         (portIndex == pin_to_timer_map[i].portIndex))
        return pin_to_timer_map[i].timerIndex;
    }
    
    // if we weren't an allowed timer, just return none.
    return NOT_ON_TIMER;
  }
}

void MultiTone(uint8_t pin, uint16_t freq)
{
  // get the tone timer associated with this pin.
  uint8_t timer = GetTimerForPin(pin);

  // if there's no tone timer, don't do anything.
  if(timer == NOT_ON_TIMER)
    return;

  // now we set up the timer.
  TC0_t* tc0 = (TC0_t*)timerToTC0(timer);
  TC1_t* tc1 = (TC1_t*)timerToTC1(timer);
#if defined(TCC2) || defined(TCD2)
  // we can't use 8 bit timers.
  if((TC2_t*)timerToTC2(timer))
    return;
#endif
  
  // if we've reached this point, we know it's a valid timer so we can calculate
  // the period.
  if(freq == 0)
  {
    noMultiTone(pin);
    return;
  }

  uint16_t period = 0;
  uint8_t prescale_ctrla = 0;

  // try to get the period and the prescale, give up if we cant.
  if(!GetPeriodAndPrescaleForFreq(period, prescale_ctrla, freq))
    return;

  // if there's no DFLL, the internal clock won't be stable for us to use.
  EnableDFLL();

  // get the channel and compare register.
  uint8_t channel = timerToChannel(timer);
  volatile uint16_t* channel_register = timerToChannelRegister(timer);

  // it's now an output pin.
  pinMode(pin, OUTPUT);

  // finally, set up the timer!
  if ( tc0 ) {
    tc0->PERBUF = period;
    tc0->CTRLA  = prescale_ctrla;
    tc0->CTRLB  = ( tc0->CTRLB & ~TC0_WGMODE_gm ) | TC_WGMODE_SINGLESLOPE_gc;
    tc0->CTRLB |= TC0_CCAEN_bm << channel;
  } else if ( tc1 ) {
    tc1->PERBUF  = period;
    tc1->CTRLA   = prescale_ctrla;
    tc1->CTRLB   = ( tc1->CTRLB & ~TC1_WGMODE_gm ) | TC_WGMODE_SINGLESLOPE_gc;
    tc1->CTRLB  |= TC1_CCAEN_bm << channel;
  } else {
    // something weird happened - if we accept the pin, it should have a valid timer.
    return;
  }

  // duty cycle is exactly 1/2 the freq.
  *channel_register = period >> 1;
}

void noMultiTone(uint8_t pin)
{
  // get the tone timer associated with this pin.
  uint8_t timer = GetTimerForPin(pin);

  // if there's no tone timer, don't do anything.
  if(timer == NOT_ON_TIMER)
    return;

  // get the timer register.
  TC0_t* tc0 = (TC0_t*)timerToTC0(timer);
  TC1_t* tc1 = (TC1_t*)timerToTC1(timer);
#if defined(TCC2) || defined(TCD2)
  // we can't use 8 bit timers.
  if((TC2_t*)timerToTC2(timer))
    return;
#endif

  uint8_t channel = timerToChannel(timer);
  // turn off the channel.
  if ( tc0 ) {
    tc0->CTRLB &= ~(TC0_CCAEN_bm << channel);
  } else if ( tc1 ) {
    tc1->CTRLB &= ~(TC1_CCAEN_bm << channel);
  }
}
