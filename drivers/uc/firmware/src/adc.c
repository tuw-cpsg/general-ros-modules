/**
 * @file
 * @author Denise Ratasich
 * @date 07.07.2015
 *
 * @brief ADC module for ATmega328P.
 **/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "adc.h"

#define NUM_CHANNELS	(8)

void adc_init(void)
{
  // use AREF, value right adjusted
  // enable ADC, set prescaler to slowest (128), no interrupt
  ADCSRA |= (1<<ADEN);
  ADCSRA |= ((1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2));
}

uint16_t adc_value(uint8_t channel)
{
  uint16_t res;

  if (channel >= NUM_CHANNELS)
    return 0;

  // set mux
  ADMUX &= 0xF0;
  ADMUX |= channel;

  // start conversion
  ADCSRA |= (1<<ADSC);

  // wait until complete
  ADCSRA |= (1<<ADSC);

  // get result
  res = ADCL;
  res |= (ADCH<<8);

  return res;
}
