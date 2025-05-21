#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "usart1.h"

#define RE_DE_PIN PD4
#define LED_PIN   PB0   // Optional TX LED
#define MAX_LEN 32
char buffer[MAX_LEN];
uint8_t idx = 0;

void blink_led() {
	PORTB |= (1 << LED_PIN);
	_delay_ms(100);
	PORTB &= ~(1 << LED_PIN);
}

int main(void) {
	DDRD |= (1 << RE_DE_PIN);   // RE/DE pin as output
	DDRB |= (1 << LED_PIN);     // TX LED pin as output
	PORTD &= ~(1 << RE_DE_PIN); // Start in receive mode

	usart1_init(9600);

	while (1) {
		if (usart1_available()) {
			char c = usart1_read();

			if (idx < MAX_LEN - 1) {
				buffer[idx++] = c;

				if (c == '\n') {
					buffer[idx] = '\0';  // Null-terminate

					// Echo full message
					UCSR1A |= (1 << TXC1);
					PORTD |= (1 << RE_DE_PIN);
					_delay_us(10);

					for (uint8_t i = 0; i < idx+1; i++) {
						usart1_write(buffer[i]);
						while (!(UCSR1A & (1 << TXC1)));
					}

					_delay_us(10);
					PORTD &= ~(1 << RE_DE_PIN);
					idx = 0;
				}
				} else {
				idx = 0; // Overflow protection
			}
		}
	}


}