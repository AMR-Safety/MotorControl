// motor.c
#include "motor.h"
#define F_CPU 16000000UL
#define STEPS_PER_REV 400  // Adjust this if using microstepping
#define PRESCALER 8

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void setup_pins(void) {
	DDRB |= (1 << STEP_PIN) | (1 << DIR_PIN) | (1 << EN_PIN);

	PORTB &= ~(1 << EN_PIN);  // Enable driver (active LOW)
	set_direction(1);         // Default to forward direction
}

float angular_velocity_to_rpm(float omega) {
	return omega * (60.0f / (2.0f * M_PI));  // ?(rad/s) × 60 / 2? ? RPM
}

uint16_t rpm_to_ocr(float rpm) {
	if (rpm <= 0) return 65535;
	float steps_per_sec = (rpm * STEPS_PER_REV) / 60.0f;
	float ocr = (float)F_CPU / (PRESCALER * steps_per_sec);
	if (ocr > 65535) ocr = 65535;
	return (uint16_t)ocr;
}

void set_angular_velocity(float omega) {
	float rpm = angular_velocity_to_rpm(omega);
	uint16_t ocr = rpm_to_ocr(rpm);
	setup_timer1(ocr);
}
	
void setup_timer1(uint16_t ocr_value) {
	TCCR1A = (1 << COM1A0);                 // Toggle OC1A on compare match
	TCCR1B = (1 << WGM12) | (1 << CS11);    // CTC mode, prescaler = 8
	OCR1A = ocr_value;
}


void set_direction(uint8_t forward) {
	if (forward) {
		PORTB |= (1 << DIR_PIN);  // Set DIR high ? forward
		} else {
		PORTB &= ~(1 << DIR_PIN); // Set DIR low ? reverse
	}
}
