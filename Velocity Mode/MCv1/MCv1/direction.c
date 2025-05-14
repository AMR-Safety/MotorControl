// direction.c
#include "direction.h"

void setup_pins(void) {
	DDRB |= (1 << STEP_PIN) | (1 << DIR_PIN) | (1 << EN_PIN);
	PORTB &= ~(1 << EN_PIN);  // Enable driver (active LOW)
	set_direction(1);         // Default direction = forward
}

void set_direction(uint8_t forward) {
	if (forward) {
		PORTB |= (1 << DIR_PIN);
		} else {
		PORTB &= ~(1 << DIR_PIN);
	}
}
