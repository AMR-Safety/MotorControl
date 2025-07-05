// velocity.c
#include "config.h"
#include "velocity.h"

volatile float target_velocity_motor1 = 0.0f;
volatile float target_velocity_motor2 = 0.0f;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float angular_velocity_to_rpm(float omega) {
	return omega * (60.0f / (2.0f * M_PI));
}

uint16_t rpm_to_ocr(float rpm) {
	if (rpm <= 0) {
		return 0;  // Return 0 to indicate motor should be stopped
	}
	
	float steps_per_sec = (rpm * STEPS_PER_REV) / 60.0f;
	
	// Avoid division by zero
	if (steps_per_sec <= 0) {
		return 0;
	}
	
	float ocr = (float)F_CPU / (PRESCALER * steps_per_sec);
	
	// Clamp to maximum timer value
	if (ocr > 65535) ocr = 65535;
	
	return (uint16_t)ocr;
}

void setup_timer1(uint16_t ocr_value) {
	if (ocr_value == 0) {
		// Stop the motor by disabling output compare
		TCCR1A &= ~(1 << COM1A0);
		TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // Stop timer
		} else {
		// Configure timer for CTC mode with toggle output
		TCCR1A = (1 << COM1A0);  // Toggle OC1A on compare match
		TCCR1B = (1 << WGM12) | (1 << CS11);  // CTC mode, prescaler 8
		OCR1A = ocr_value;
	}
}

void setup_timer3(uint16_t ocr_value) {
	if (ocr_value == 0) {
		// Stop the motor by disabling output compare
		TCCR3A &= ~(1 << COM3A0);
		TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30)); // Stop timer
		} else {
		// Configure timer for CTC mode with toggle output
		TCCR3A = (1 << COM3A0);  // Toggle OC3A on compare match
		TCCR3B = (1 << WGM32) | (1 << CS31);  // CTC mode, prescaler 8
		OCR3A = ocr_value;
	}
}

void set_angular_velocity_motor1(float omega) {
	target_velocity_motor1 = omega;  // Store target velocity
	float rpm = angular_velocity_to_rpm(omega);
	uint16_t ocr = rpm_to_ocr(rpm);
	setup_timer1(ocr);
}

void set_angular_velocity_motor2(float omega) {
	target_velocity_motor2 = omega;  // Store target velocity
	float rpm = angular_velocity_to_rpm(omega);
	uint16_t ocr = rpm_to_ocr(rpm);
	setup_timer3(ocr);
}

/**
 * Smoothly steps toward a target angular velocity.
 * Call this every 100 ms to gradually ramp up/down to the new velocity.
 *
 * @param current_omega     Current angular velocity [rad/s]
 * @param target_omega      Desired target angular velocity [rad/s]
 * @param step_size         How much to change per 100 ms [rad/s]
 * @return                  Updated angular velocity to apply
 */
float step_toward_velocity(float current_omega, float target_omega, float step_size) {
    float delta = target_omega - current_omega;

    if (fabs(delta) <= step_size) {
        return target_omega;  // Close enough; jump to target
    }

    if (delta > 0) {
        return current_omega + step_size;  // Ramp up
    } else {
        return current_omega - step_size;  // Ramp down
    }
}

