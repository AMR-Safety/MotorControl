#include "motion_control.h"
#include "velocity.h"
#include "direction.h"
#include "trapezoid.h"
#include "scurve.h"
#include "config.h"
#include <util/delay.h>
#include <math.h>
#include <stdint.h>  // for uint16_t

#define WHEEL_RADIUS 0.062f
#define STEP_ANGLE   (2.0f * M_PI / STEPS_PER_REV)

// Global step counters for precise movement
volatile uint32_t motor1_steps = 0;
volatile uint32_t motor2_steps = 0;

// Improved delay function with overflow protection
void variable_delay_ms(uint32_t ms) {
	while (ms > 0) {
		uint16_t chunk = (ms > 65535) ? 65535 : ms;
		for (uint16_t i = 0; i < chunk; i++) {
			_delay_ms(1);
		}
		ms -= chunk;
	}
}

void move_distance(float distance_m, float omega_target, uint8_t use_trapezoid) {
	float wheel_circ = 2.0f * M_PI * WHEEL_RADIUS;
	const float GEAR_RATIO = 10.0f; // 10:1 gear ratio
	
	// Calculate distance covered during acceleration and deceleration phases
	uint16_t profile_steps = (use_trapezoid) ? TRAPEZOID_STEPS : SCURVE_STEPS;
	float accel_distance = calculate_accel_distance(omega_target, profile_steps);
	float decel_distance = accel_distance; // Same as acceleration distance
	
	// Calculate cruise distance (remaining distance after accel/decel)
	float cruise_distance = distance_m - accel_distance - decel_distance;
	
	// Ensure cruise distance is not negative
	if (cruise_distance < 0) {
		cruise_distance = 0;
		// For very short distances, you might want to use a different profile
		// or adjust the acceleration parameters
	}
	
	// Convert cruise distance to physical motor steps
	// Account for gear ratio: motor rotations = wheel rotations * gear ratio
	float wheel_rotations = cruise_distance / wheel_circ;
	float motor_rotations = wheel_rotations * GEAR_RATIO;
	uint32_t cruise_steps = (uint32_t)(motor_rotations * STEPS_PER_REV);
	
	// Reset step counters
	motor1_steps = 0;
	motor2_steps = 0;
	
	// Ramp Up
	if (use_trapezoid) {
		ramp_up_trapezoid(omega_target);
		} else {
		ramp_up_scurve(omega_target);
	}
	
	// Cruise phase with precise step counting
	set_angular_velocity_motor1(omega_target);
	set_angular_velocity_motor2(omega_target);
	
	uint32_t target_steps = cruise_steps;
	while (motor1_steps < target_steps && motor2_steps < target_steps) {
		// Wait for timer interrupts to generate steps
		// This is more precise than delay-based timing
		if (TIFR1 & (1 << OCF1A)) {
			TIFR1 |= (1 << OCF1A);
			motor1_steps++;
		}
		if (TIFR3 & (1 << OCF3A)) {
			TIFR3 |= (1 << OCF3A);
			motor2_steps++;
		}
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

// Helper function to calculate distance covered during acceleration
float calculate_accel_distance(float omega_target, uint16_t profile_steps) {
	float total_distance = 0;
	//float wheel_circ = 2.0f * M_PI * WHEEL_RADIUS;
	const float GEAR_RATIO = 10.0f; // 10:1 gear ratio
	
	// Calculate distance for each step of the acceleration profile
	for (int i = 1; i <= profile_steps; i++) {
		float motor_omega = (omega_target * i) / profile_steps;
		// Convert motor angular velocity to wheel angular velocity
		float wheel_omega = motor_omega / GEAR_RATIO;
		// Distance = velocity * time
		// Convert wheel omega (rad/s) to linear velocity (m/s): v = omega * radius
		float linear_velocity = wheel_omega * WHEEL_RADIUS;
		float step_distance = linear_velocity * (TRAPEZOID_DELAY_MS / 1000.0f); // Convert ms to s
		total_distance += step_distance;
	}
	
	return total_distance;
}

#define WHEELBASE 0.46f  // distance between left and right wheels (in meters)

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

void drive_differential(float vel, float omega) {
	float w_left  = (vel - (omega * WHEELBASE / 2.0f)) / WHEEL_RADIUS;
	float w_right = (vel + (omega * WHEELBASE / 2.0f)) / WHEEL_RADIUS;

	if (w_left >= 0) set_direction_motor1(1); else set_direction_motor1(0);
	if (w_right >= 0) set_direction_motor2(1); else set_direction_motor2(0);

	set_angular_velocity_motor1(fabsf(w_left));
	set_angular_velocity_motor2(fabsf(w_right));
}
