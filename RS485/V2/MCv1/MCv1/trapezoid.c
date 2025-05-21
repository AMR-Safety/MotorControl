//trapizoid.c
#include "config.h"
#include "trapezoid.h"
#include "velocity.h"

#include <util/delay.h>

void ramp_up_trapezoid(float target_omega) {
	for (int i = 1; i <= TRAPEZOID_STEPS; i++) {
		float omega = (target_omega * i) / TRAPEZOID_STEPS;
		set_angular_velocity(omega);
		set_angular_velocity_motor2(omega);
		_delay_ms(TRAPEZOID_DELAY_MS);
	}
}

void ramp_down_trapezoid(float target_omega) {
	for (int i = TRAPEZOID_STEPS - 1; i >= 0; i--) {
		float omega = (target_omega * i) / TRAPEZOID_STEPS;
		set_angular_velocity(omega);
		set_angular_velocity_motor2(omega);
		_delay_ms(TRAPEZOID_DELAY_MS);
	}
}
