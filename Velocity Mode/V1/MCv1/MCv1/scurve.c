// scurve.c
#define F_CPU 16000000UL
#include "scurve.h"
#include "velocity.h"
#include <util/delay.h>
#include <math.h>

#define SCURVE_STEPS 50
#define SCURVE_DELAY_MS 50

static float s_curve_profile(float x) {
	// S-curve shaping function: 3x^2 - 2x^3
	return 3 * x * x - 2 * x * x * x;
}

void run_scurve_velocity(float target_omega) {
	for (int i = 1; i <= SCURVE_STEPS; i++) {
		float x = (float)i / SCURVE_STEPS;
		float omega = s_curve_profile(x) * target_omega;
		set_angular_velocity(omega);
		_delay_ms(SCURVE_DELAY_MS);
	}
	// Maintain final velocity
	set_angular_velocity(target_omega);
}
