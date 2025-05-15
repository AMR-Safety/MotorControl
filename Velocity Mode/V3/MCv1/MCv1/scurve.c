// scurve.c
#include "config.h"
#include "scurve.h"
#include "velocity.h"

#include <util/delay.h>
#include <math.h>

static float s_curve_profile(float x) {
	return 3 * x * x - 2 * x * x * x;
}

void ramp_up_scurve(float target_omega) {
	for (int i = 1; i <= SCURVE_STEPS; i++) {
		float x = (float)i / SCURVE_STEPS;
		float omega = s_curve_profile(x) * target_omega;
		set_angular_velocity(omega);
		set_angular_velocity_motor2(omega);
		_delay_ms(SCURVE_DELAY_MS);
	}
}

void ramp_down_scurve(float target_omega) {
	for (int i = SCURVE_STEPS - 1; i >= 0; i--) {
		float x = (float)i / SCURVE_STEPS;
		float omega = s_curve_profile(x) * target_omega;
		set_angular_velocity(omega);
		set_angular_velocity_motor2(omega);
		_delay_ms(SCURVE_DELAY_MS);
	}
}
