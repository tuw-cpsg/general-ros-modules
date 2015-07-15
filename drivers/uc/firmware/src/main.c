/**
 * @file
 * @author Denise Ratasich
 * @date 06.07.2015
 *
 * @brief Firmware for ATmega328P on Gertboard providing sensor values
 * from AD converter.
 **/

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 12000000UL
#include <util/delay.h>

#include "adc.h"
#include "uart.h"

/** Sensors connected to ADC channels. */
#define ADC_INPUTS_NUM 1

uint16_t adcs[ADC_INPUTS_NUM];

/** Entry point of program. */
int main(void)
{
  uint8_t i;

  adc_init();
  uart_init();

  uart_println_b("init done");
  
  while (1) {
    // sample adc values
    for (i = 0; i < ADC_INPUTS_NUM; i++) {
      adcs[i] = adc_value(i);
    }

    // send sensor values
    uart_putc_b('S'); // start signal
    uart_putc_b(ADC_INPUTS_NUM*2); // number of bytes to send
    for (i = 0; i < ADC_INPUTS_NUM; i++) {
      uart_putc_b(adcs[i]>>8);
      uart_putc_b(adcs[i]);
    }
    uart_putc_b('E'); // end signal
    uart_println_b("");

    // for RPi sync
    _delay_us(500);
  }

  return 0;
}
