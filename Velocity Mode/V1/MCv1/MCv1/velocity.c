// velocity.c
#include "velocity.h"
#define F_CPU 16000000UL
#define STEPS_PER_REV 400
#define PRESCALER 8

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float angular_velocity_to_rpm(float omega) {
	return omega * (60.0f / (2.0f * M_PI));
}

uint16_t rpm_to_ocr(float rpm) {
	if (rpm <= 0) return 65535;
	float steps_per_sec = (rpm * STEPS_PER_REV) / 60.0f;
	float ocr = (float)F_CPU / (PRESCALER * steps_per_sec);
	if (ocr > 65535) ocr = 65535;
	return (uint16_t)ocr;
}

void setup_timer1(uint16_t ocr_value) {
	TCCR1A = (1 << COM1A0);                 // Toggle OC1A on compare match
	TCCR1B = (1 << WGM12) | (1 << CS11);    // CTC mode, prescaler = 8
	OCR1A = ocr_value;
}

void set_angular_velocity(float omega) {
	float rpm = angular_velocity_to_rpm(omega);
	uint16_t ocr = rpm_to_ocr(rpm);
	setup_timer1(ocr);
}
