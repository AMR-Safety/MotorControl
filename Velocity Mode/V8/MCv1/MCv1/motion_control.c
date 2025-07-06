#include "motion_control.h"
#include "velocity.h"
#include "direction.h"
#include "trapezoid.h"
#include "scurve.h"
#include "config.h"
#include <util/delay.h>
#include <math.h>
#include <stdint.h>  // for uint16_t


#define STEP_ANGLE   (2.0f * M_PI / STEPS_PER_REV)

// Global step counters for precise movement
volatile uint32_t motor1_steps = 0;
volatile uint32_t motor2_steps = 0;


void diagnostic_move() {
	// Move exactly 1 wheel rotation worth of steps
	float wheel_circ = 2.0f * M_PI * 0.062f;  // Should be ~0.3896m
	
	// Calculate steps for exactly 1 wheel rotation
	uint32_t steps_for_one_rotation = STEPS_PER_REV * 10;  // 800 * 10 = 8000 steps
	
	motor1_steps = 0;
	motor2_steps = 0;
	
	set_angular_velocity_motor1(52.36f);  // Moderate speed
	set_angular_velocity_motor2(52.36f);
	
	while (motor1_steps < steps_for_one_rotation && motor2_steps < steps_for_one_rotation) {
		if (TIFR1 & (1 << OCF1A)) {
			TIFR1 |= (1 << OCF1A);
			motor1_steps++;
		}
		if (TIFR3 & (1 << OCF3A)) {
			TIFR3 |= (1 << OCF3A);
			motor2_steps++;
		}
	}
	
	set_angular_velocity_motor1(0);
	set_angular_velocity_motor2(0);
	
	// This should move exactly 38.96cm if everything is calibrated correctly
}


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
	if (distance_m <= 0.001f) {
		return;
	}
	
	float wheel_circ = 2.0f * M_PI * WHEEL_RADIUS;
	const float GEAR_RATIO = 10.0f;
	
	uint16_t profile_steps = (use_trapezoid) ? TRAPEZOID_STEPS : SCURVE_STEPS;
	float accel_distance = calculate_accel_distance(omega_target, profile_steps);
	float decel_distance = accel_distance;
	float total_ramp_distance = accel_distance + decel_distance;
	
	// For very small distances, use constant speed
	if (distance_m < total_ramp_distance * 0.5f) {  // Less than half the ramp distance
		move_constant_speed(distance_m, omega_target * 0.3f);  // 30% of max speed
		return;
	}
	
	// For small but not tiny distances, use scaled trapezoid
	if (distance_m < total_ramp_distance) {
		// Use your existing scaled trapezoid code
		float scale_factor = distance_m / total_ramp_distance;
		float scaled_omega = omega_target * scale_factor;
		
		if (use_trapezoid) {
			ramp_up_trapezoid(scaled_omega);
			ramp_down_trapezoid(scaled_omega);
		}
		return;
	}
	// Normal profile for longer distances
	float cruise_distance = distance_m - accel_distance - decel_distance;
	
	// Convert cruise distance to steps
	float wheel_rotations = cruise_distance / wheel_circ;
	float motor_rotations = wheel_rotations * GEAR_RATIO;
	uint32_t cruise_steps = (uint32_t)(motor_rotations * STEPS_PER_REV);
	
	// Reset step counters
	motor1_steps = 0;
	motor2_steps = 0;
	
	// Execute normal trapezoid profile
	if (use_trapezoid) {
		ramp_up_trapezoid(omega_target);
		} else {
		ramp_up_scurve(omega_target);
	}
	
	// Cruise phase
	set_angular_velocity_motor1(omega_target);
	set_angular_velocity_motor2(omega_target);
	
	while (motor1_steps < cruise_steps && motor2_steps < cruise_steps) {
		if (TIFR1 & (1 << OCF1A)) {
			TIFR1 |= (1 << OCF1A);
			motor1_steps++;
		}
		if (TIFR3 & (1 << OCF3A)) {
			TIFR3 |= (1 << OCF3A);
			motor2_steps++;
		}
	}
	
	// Ramp down
	if (use_trapezoid) {
		ramp_down_trapezoid(omega_target);
		} else {
		ramp_down_scurve(omega_target);
	}
	
	// Stop motors
	set_angular_velocity_motor1(0);
	set_angular_velocity_motor2(0);
}

void move_constant_speed(float distance_m, float omega) {
	float wheel_circ = 2.0f * M_PI * WHEEL_RADIUS;
	const float GEAR_RATIO = 10.0f;
	
	float wheel_rotations = distance_m / wheel_circ;
	float motor_rotations = wheel_rotations * GEAR_RATIO;
	uint32_t target_steps = (uint32_t)(motor_rotations * STEPS_PER_REV);
	
	motor1_steps = 0;
	motor2_steps = 0;
	
	set_angular_velocity_motor1(omega);
	set_angular_velocity_motor2(omega);
	
	while (motor1_steps < target_steps) {
		if (TIFR1 & (1 << OCF1A)) {
			TIFR1 |= (1 << OCF1A);
			motor1_steps++;
		}
	}
	
	set_angular_velocity_motor1(0);
	set_angular_velocity_motor2(0);
}

/*// Helper function to calculate distance covered during acceleration
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
*/

float calculate_accel_distance(float omega_target, uint16_t profile_steps) {
	float total_distance = 0;
	const float GEAR_RATIO = 10.0f;
	
	// Calculate distance for each step of the acceleration profile
	for (int i = 1; i <= profile_steps; i++) {
		// Current step velocity
		float motor_omega_current = (omega_target * i) / profile_steps;
		// Previous step velocity
		float motor_omega_previous = (omega_target * (i-1)) / profile_steps;
		
		// Use average velocity for this step
		float motor_omega_avg = (motor_omega_current + motor_omega_previous) / 2.0f;
		
		// Convert to wheel velocity
		float wheel_omega_avg = motor_omega_avg / GEAR_RATIO;
		
		// Distance = average_velocity * time
		float linear_velocity_avg = wheel_omega_avg * WHEEL_RADIUS;
		float step_distance = linear_velocity_avg * (TRAPEZOID_DELAY_MS / 1000.0f);
		total_distance += step_distance;
	}
	
	return total_distance;
}

// Turn Angle

void turn_angle(float angle_rad, float omega_target, uint8_t use_trapezoid) {
	if (fabsf(angle_rad) <= 0.001f) {  // ~0.06 degrees threshold
		return;
	}
	
	// Calculate arc distance each wheel travels
	float arc = (WHEELBASE / 2.0f) * fabsf(angle_rad);
	float wheel_circ = 2.0f * M_PI * WHEEL_RADIUS;
	const float GEAR_RATIO = 10.0f;
	
	// Convert to steps (including gear ratio)
	float wheel_rotations = arc / wheel_circ;
	float motor_rotations = wheel_rotations * GEAR_RATIO;
	uint32_t total_steps = (uint32_t)(motor_rotations * STEPS_PER_REV);
	
	// Calculate ramp distances
	uint16_t profile_steps = (use_trapezoid) ? TRAPEZOID_STEPS : SCURVE_STEPS;
	float accel_distance = calculate_accel_distance(omega_target, profile_steps);
	float decel_distance = accel_distance;
	float total_ramp_distance = accel_distance + decel_distance;
	
	// Set directions based on turn direction
	if (angle_rad > 0) {  // Positive = turn left
		set_direction_motor1(0);  // Left wheel forward
		set_direction_motor2(1);  // Right wheel backward
		} else {  // Negative = turn right
		set_direction_motor1(1);  // Left wheel backward
		set_direction_motor2(0);  // Right wheel forward
	}
	
	// Handle small angles (similar to small distances)
	if (arc < total_ramp_distance * 0.5f) {
		// Very small angle - use constant speed
		turn_constant_speed(total_steps, omega_target * 0.3f);
		return;
	}
	
	if (arc < total_ramp_distance) {
		// Small angle - use scaled trapezoid
		float scale_factor = arc / total_ramp_distance;
		float scaled_omega = omega_target * scale_factor;
		
		if (use_trapezoid) {
			ramp_up_trapezoid(scaled_omega);
			ramp_down_trapezoid(scaled_omega);
			} else {
			ramp_up_scurve(scaled_omega);
			ramp_down_scurve(scaled_omega);
		}
		return;
	}
	
	// Normal turn for larger angles
	// Calculate ramp steps inline
	float ramp_wheel_rotations = accel_distance / wheel_circ;
	float ramp_motor_rotations = ramp_wheel_rotations * GEAR_RATIO;
	uint32_t ramp_steps = (uint32_t)(ramp_motor_rotations * STEPS_PER_REV);
	
	uint32_t cruise_steps = total_steps - (2 * ramp_steps);
	
	// Reset step counters
	motor1_steps = 0;
	motor2_steps = 0;
	
	// Ramp Up
	if (use_trapezoid) {
		ramp_up_trapezoid(omega_target);
		} else {
		ramp_up_scurve(omega_target);
	}
	
	// Cruise phase with proper step counting
	set_angular_velocity_motor1(omega_target);
	set_angular_velocity_motor2(omega_target);
	
	while (motor1_steps < cruise_steps && motor2_steps < cruise_steps) {
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

void turn_constant_speed(uint32_t target_steps, float omega) {
	motor1_steps = 0;
	motor2_steps = 0;
	
	set_angular_velocity_motor1(omega);
	set_angular_velocity_motor2(omega);
	
	while (motor1_steps < target_steps && motor2_steps < target_steps) {
		if (TIFR1 & (1 << OCF1A)) {
			TIFR1 |= (1 << OCF1A);
			motor1_steps++;
		}
		if (TIFR3 & (1 << OCF3A)) {
			TIFR3 |= (1 << OCF3A);
			motor2_steps++;
		}
	}
	
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
