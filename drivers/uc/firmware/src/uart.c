/**
 * @file
 * @author Denise Ratasich
 * @date 08.07.2015
 *
 * @brief UART driver for ATmega328P.
 *
 * Used for debugging.
 **/

#include <avr/interrupt.h>
#include "uart.h"

static volatile uint8_t uart_receive_flag = 0;
static volatile char uart_receive_data;

void uart_init(void)
{
  // pin operations of Tx and Rx are overriden when in UART enabled

  // set baud rate: 115200
  UBRR0H = 0;
  UBRR0L = 12;

  // double speed, buffer ready to be written
  UCSR0A |= (1<<U2X0) | (1<<UDRE0);

  // already default value of UART register C:
  //UCSR0C &= ~(1<<UMSEL00) & ~(1<<UMSEL01);// asynchronous mode
  //UCSR0C &= ~(1<<UPM00) & ~(1<<UPM01);// no parity
  //UCSR0C &= ~(1<<USBS0); // 1 stopbit
  //UCSR0B &= ~(1<<UCSZ02);
  //UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);// 8-bit

  // enable transmit/receive, receive-interrupt
  UCSR0B |= (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0);
}

void uart_putc_b(char myData)
{
  // wait for empty transmit buffer
  while ( !( UCSR0A & (1<<UDRE0)) );
  UDR0 = myData;
}

void uart_print_b(char* string)
{
  while(*string != '\0') {
    uart_putc_b(*string);
    string++;
  }
}

void uart_println_b(char* string)
{
  uart_print_b(string);
  uart_putc_b('\n');
  uart_putc_b('\r');
}

void uart_printUInt8_b(uint8_t value)
{
  uart_putc_b(value/100 + '0');
  value %= 100;
  uart_putc_b(value/10 + '0');
  uart_putc_b(value%10 + '0');
}

void uart_printUInt16_b(uint16_t value)
{
  uart_putc_b(value/10000 + '0');
  value %= 10000;
  uart_putc_b(value/1000 + '0');
  value %= 1000;
  uart_putc_b(value/100 + '0');
  value %= 100;
  uart_putc_b(value/10 + '0');
  uart_putc_b(value%10 + '0');
}

// receive complete
ISR(USART_RX_vect)
{
  uart_receive_data = UDR0;
  uart_receive_flag = 1;
}

uint8_t uart_getc(char* data)
{
  if(uart_receive_flag) {
    *data = uart_receive_data;
    uart_receive_flag = 0;
    return 1;
  }

  return 0;
}
