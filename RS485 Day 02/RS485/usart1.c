#define F_CPU 16000000UL
#include "usart1.h"

void usart1_init(uint16_t baud) {
	uint16_t ubrr = F_CPU / 16 / baud - 1;
	UBRR1H = (ubrr >> 8);
	UBRR1L = ubrr;
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

void usart1_write(char data) {
	while (!(UCSR1A & (1 << UDRE1)));
	UDR1 = data;
}

char usart1_read(void) {
	while (!(UCSR1A & (1 << RXC1)));
	return UDR1;
}

uint8_t usart1_available(void) {
	return (UCSR1A & (1 << RXC1));
}
