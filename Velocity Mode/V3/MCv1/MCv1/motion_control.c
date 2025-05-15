#include "motion_control.h"
#include "velocity.h"
#include "direction.h"
#include "trapezoid.h"
#include "scurve.h"
#include "config.h"
#include <util/delay.h>
#include <math.h>

#define WHEEL_RADIUS 0.03f
#define STEP_ANGLE   (2.0f * M_PI / STEPS_PER_REV)

void move_distance(float distance_m, float omega_target, uint8_t use_trapezoid) {
	float wheel_circ = 2.0f * M_PI * WHEEL_RADIUS;
	float rotations = distance_m / wheel_circ;
	uint32_t total_steps = (uint32_t)(rotations * STEPS_PER_REV);

	uint16_t accel_steps = (use_trapezoid) ? TRAPEZOID_STEPS : SCURVE_STEPS;
	uint32_t cruise_steps = (total_steps > 2 * accel_steps) ? (total_steps - 2 * accel_steps) : 0;

	// Ramp Up
	if (use_trapezoid) {
		ramp_up_trapezoid(omega_target);
		} else {
		ramp_up_scurve(omega_target);
	}

	// Cruise at constant speed
	set_angular_velocity(omega_target);
	set_angular_velocity_motor2(omega_target);

	for (uint32_t i = 0; i < cruise_steps; i++) {
		while (!(TIFR1 & (1 << OCF1A))) {}
		TIFR1 |= (1 << OCF1A);
		while (!(TIFR3 & (1 << OCF3A))) {}
		TIFR3 |= (1 << OCF3A);
	}

	// Ramp Down
	if (use_trapezoid) {
		ramp_down_trapezoid(omega_target);
		} else {
		ramp_down_scurve(omega_target);
	}

	// Stop motors
	set_angular_velocity(0);
	set_angular_velocity_motor2(0);
}
