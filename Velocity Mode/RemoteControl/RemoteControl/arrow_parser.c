#include "arrow_parser.h"
#include <string.h>
#include "motion_control.h"

#define RE_DE_PIN   PD4
#define RS485_DDR   DDRD
#define RS485_PORT  PORTD
#define BUFFER_SIZE 8

static char buffer[BUFFER_SIZE];
static uint8_t idx = 0;

static void USART1_init(uint16_t baud) {
	uint16_t ubrr = (F_CPU / (16UL * baud)) - 1;
	UBRR1H = (ubrr >> 8);
	UBRR1L = ubrr;
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 8N1
}

static void RS485_send_char(char c) {
	while (!(UCSR1A & (1 << UDRE1)));
	UDR1 = c;
}

void RS485_send_str(const char* str) {
	RS485_PORT |= (1 << RE_DE_PIN);  // Enable transmit
	_delay_ms(1);
	while (*str) {
		RS485_send_char(*str++);
	}
	_delay_ms(1);
	RS485_PORT &= ~(1 << RE_DE_PIN); // Back to receive
}

void ArrowParser_init(void) {
	RS485_DDR |= (1 << RE_DE_PIN);
	RS485_PORT &= ~(1 << RE_DE_PIN); // Start in receive mode
	USART1_init(9600);
	idx = 0;
}

void ArrowParser_process_input(void) {
	if (UCSR1A & (1 << RXC1)) {
		char c = UDR1;

		if (idx < BUFFER_SIZE - 1) {
			buffer[idx++] = c;
			buffer[idx] = '\0';
		}

		if (idx >= 3 && buffer[idx - 3] == 27 && buffer[idx - 2] == '[') {
			switch (buffer[idx - 1]) {
				case 'A':
				RS485_send_str("up\n");
				set_direction_motor1(0);
				set_direction_motor2(0);
				move_distance(5, 104.72, 1);
				break;
				case 'B':
				RS485_send_str("down\n");
				set_direction_motor1(1);
				set_direction_motor2(1);
				move_distance(5, 104.72, 1);
				break;
				case 'C':
				RS485_send_str("right\n");
				set_direction_motor1(0);
				set_direction_motor2(1);
				move_distance(5, 104.72, 1);
				break;
				case 'D':
				RS485_send_str("left\n");
				set_direction_motor1(1);
				set_direction_motor2(0);
				move_distance(5, 104.72, 1);
				break;
			}
			idx = 0;
		}

		if (idx >= BUFFER_SIZE - 1) {
			idx = 0;
		}
	}
}
