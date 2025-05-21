#ifndef USART1_H
#define USART1_H

#include <avr/io.h>
#include <stdint.h>

void usart1_init(uint16_t baud);
void usart1_write(char data);
char usart1_read(void);
uint8_t usart1_available(void);

#endif
