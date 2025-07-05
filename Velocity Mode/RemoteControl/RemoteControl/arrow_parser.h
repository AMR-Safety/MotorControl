#ifndef ARROW_PARSER_H
#define ARROW_PARSER_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

void ArrowParser_init(void);
void ArrowParser_process_input(void);
void RS485_send_str(const char* str); // Optional if used elsewhere

#endif
