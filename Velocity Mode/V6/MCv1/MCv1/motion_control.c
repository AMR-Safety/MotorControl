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
	set_angular_velocity_motor1(omega_target);
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
	set_angular_velocity_motor1(0);
	set_angular_velocity_motor2(0);
}

#define WHEELBASE 0.12f  // distance between left and right wheels (in meters)

void turn_angle(float angle_rad, float omega_target, uint8_t use_trapezoid) {
	float arc = (WHEELBASE / 2.0f) * fabsf(angle_rad);  // distance each wheel travels
	float wheel_circ = 2.0f * M_PI * WHEEL_RADIUS;
	float rotations = arc / wheel_circ;
	uint32_t total_steps = (uint32_t)(rotations * STEPS_PER_REV);

	uint16_t accel_steps = (use_trapezoid) ? TRAPEZOID_STEPS : SCURVE_STEPS;
	uint32_t cruise_steps = (total_steps > 2 * accel_steps) ? (total_steps - 2 * accel_steps) : 0;

	// Set directions: left forward, right reverse OR vice versa
	if (angle_rad > 0) {
		set_direction_motor1(1);         // Left wheel forward
		set_direction_motor2(0);  // Right wheel backward
		} else {
		set_direction_motor1(0);         // Left wheel backward
		set_direction_motor2(1);  // Right wheel forward
	}

	// Ramp Up
	if (use_trapezoid) {
		ramp_up_trapezoid(omega_target);
		} else {
		ramp_up_scurve(omega_target);
	}

	// Cruise at target speed
	set_angular_velocity_motor1(omega_target);
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

	// Stop
	set_angular_velocity_motor1(0);
	set_angular_velocity_motor2(0);
}

#define WHEEL_RADIUS 0.03f  // in meters
#define WHEELBASE    0.12f  // distance between wheels (meters)

void drive_differential(float vel, float omega) {
	float w_left  = (vel - (omega * WHEELBASE / 2.0f)) / WHEEL_RADIUS;
	float w_right = (vel + (omega * WHEELBASE / 2.0f)) / WHEEL_RADIUS;

	if (w_left >= 0) set_direction_motor1(1); else set_direction_motor1(0);
	if (w_right >= 0) set_direction_motor2(1); else set_direction_motor2(0);

	set_angular_velocity_motor1(fabsf(w_left));
	set_angular_velocity_motor2(fabsf(w_right));
}
