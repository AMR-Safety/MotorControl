// direction.c
#include "config.h"
#include "direction.h"



void setup_pins_motor1(void) {
	DDRB |= (1 << STEP_PIN) | (1 << DIR_PIN) | (1 << EN_PIN);
	PORTB &= ~(1 << EN_PIN);
	//set_direction_motor1(1);
}

void set_direction_motor1(uint8_t forward) {
	if (forward)
	PORTB |= (1 << DIR_PIN);
	else
	PORTB &= ~(1 << DIR_PIN);
}

void setup_pins_motor2(void) {
	DDRC |= (1 << STEP2_PIN);
	DDRB |= (1 << DIR2_PIN);
	DDRD |= (1 << EN2_PIN);
	PORTD &= ~(1 << EN2_PIN);  // Enable motor 2
	//set_direction_motor2(1);
}

void set_direction_motor2(uint8_t forward) {
	if (forward)
	PORTB |= (1 << DIR2_PIN);
	else
	PORTB &= ~(1 << DIR2_PIN);
}
