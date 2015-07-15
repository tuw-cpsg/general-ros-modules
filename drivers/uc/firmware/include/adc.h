/**
 * @file
 * @author Denise Ratasich
 * @date 07.07.2015
 *
 * @brief Header for ADC module of ATmega328P.
 **/

#ifndef __ADC_H__
#define __ADC_H__

/**
 * @brief Initializes ADC and saves channels to convert.
 *
 * Operates in single conversion mode. Prescaler is set to 128.
 **/
void adc_init(void);

/**
 * @brief Samples an ADC value.
 *
 * @param channel The channel to read.
 * @return Conversion result.
 **/
uint16_t adc_value(uint8_t channel);

#endif // __ADC_H__
