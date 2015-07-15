/**
 * @file
 * @author Denise Ratasich
 * @date 08.07.2015
 *
 * @brief Header for UART driver for ATmega328P.
 **/

#ifndef __UART_H__
#define __UART_H__

#include <avr/io.h>

void uart_init(void);
void uart_putc_b(char myData);
void uart_print_b(char* string);
void uart_println_b(char* string);
void uart_printUInt8_b(uint8_t value);
void uart_printUInt16_b(uint16_t value);
ISR(USART_RX_vect);
uint8_t uart_getc(char* data);

#endif // __UART_H__
