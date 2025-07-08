#include "motion_control.h"
#include "velocity.h"
#include "direction.h"
#include "trapezoid.h"
#include "scurve.h"
#include "config.h"
#include <util/delay.h>
#include <math.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define STEP_ANGLE   (2.0f * M_PI / STEPS_PER_REV)

// Global step counters for precise movement (kept for discrete functions)
volatile uint32_t motor1_steps = 0;
volatile uint32_t motor2_steps = 0;

// Continuous control state variables
static volatile motion_state_t current_motion_state = MOTION_STOPPED;
static volatile uint16_t motion_timeout_counter = 0;
static volatile uint16_t motion_timeout_limit = 500; // 500ms default timeout
static volatile uint8_t motion_timeout_enabled = 1;

// Speed settings
static float current_forward_speed = 52.36f;  // Default moderate speed (rad/s)
static float current_turn_speed = 50.0f;      // Default turn speed (rad/s)

// Smooth acceleration settings
static uint8_t smooth_accel_enabled = 0;
static float acceleration_rate = 20.0f;       // rad/s per acceleration step
static float target_speed_motor1 = 0.0f;
static float target_speed_motor2 = 0.0f;
static float actual_speed_motor1 = 0.0f;
static float actual_speed_motor2 = 0.0f;

// Timer interrupt for safety timeout (using Timer0 instead of Timer2)
ISR(TIMER0_COMPA_vect) {
	if (motion_timeout_enabled && current_motion_state != MOTION_STOPPED) {
		motion_timeout_counter++;
		if (motion_timeout_counter >= motion_timeout_limit) {
			emergency_stop();
		}
	}
	
	// Handle smooth acceleration if enabled
	if (smooth_accel_enabled) {
		// Gradually adjust actual speeds toward target speeds
		if (actual_speed_motor1 < target_speed_motor1) {
			actual_speed_motor1 += acceleration_rate;
			if (actual_speed_motor1 > target_speed_motor1) {
				actual_speed_motor1 = target_speed_motor1;
			}
			} else if (actual_speed_motor1 > target_speed_motor1) {
			actual_speed_motor1 -= acceleration_rate;
			if (actual_speed_motor1 < target_speed_motor1) {
				actual_speed_motor1 = target_speed_motor1;
			}
		}
		
		if (actual_speed_motor2 < target_speed_motor2) {
			actual_speed_motor2 += acceleration_rate;
			if (actual_speed_motor2 > target_speed_motor2) {
				actual_speed_motor2 = target_speed_motor2;
			}
			} else if (actual_speed_motor2 > target_speed_motor2) {
			actual_speed_motor2 -= acceleration_rate;
			if (actual_speed_motor2 < target_speed_motor2) {
				actual_speed_motor2 = target_speed_motor2;
			}
		}
		
		// Apply the gradually changing speeds
		set_angular_velocity_motor1(actual_speed_motor1);
		set_angular_velocity_motor2(actual_speed_motor2);
	}
}

void motion_control_init(void) {
	// Initialize Timer0 for 1ms interrupts (safety timeout and smooth acceleration)
	TCCR0A = (1 << WGM01);  // CTC mode
	TCCR0B = (1 << CS02) | (1 << CS00);  // Prescaler 1024
	OCR0A = (F_CPU / (1024UL * 1000UL)) - 1;  // 1ms interrupt
	TIMSK0 |= (1 << OCIE0A);  // Enable compare interrupt
	
	sei();  // Enable global interrupts
	
	current_motion_state = MOTION_STOPPED;
	motion_timeout_counter = 0;
	actual_speed_motor1 = 0.0f;
	actual_speed_motor2 = 0.0f;
	target_speed_motor1 = 0.0f;
	target_speed_motor2 = 0.0f;
}

void start_continuous_forward(float speed) {
	if (speed < 0) speed = current_forward_speed;
	
	current_motion_state = MOTION_FORWARD;
	reset_motion_timeout();
	
	set_direction_motor1(0);  // Forward
	set_direction_motor2(0);  // Forward
	
	if (smooth_accel_enabled) {
		target_speed_motor1 = speed;
		target_speed_motor2 = speed;
		} else {
		set_angular_velocity_motor1(speed);
		set_angular_velocity_motor2(speed);
	}
}

void start_continuous_backward(float speed) {
	if (speed < 0) speed = current_forward_speed;
	
	current_motion_state = MOTION_BACKWARD;
	reset_motion_timeout();
	
	set_direction_motor1(1);  // Backward
	set_direction_motor2(1);  // Backward
	
	if (smooth_accel_enabled) {
		target_speed_motor1 = speed;
		target_speed_motor2 = speed;
		} else {
		set_angular_velocity_motor1(speed);
		set_angular_velocity_motor2(speed);
	}
}

void start_continuous_turn_left(float turn_speed) {
	if (turn_speed < 0) turn_speed = current_turn_speed;
	
	current_motion_state = MOTION_TURN_LEFT;
	reset_motion_timeout();
	
	set_direction_motor1(1);  // Left wheel backward
	set_direction_motor2(0);  // Right wheel forward
	
	if (smooth_accel_enabled) {
		target_speed_motor1 = turn_speed;
		target_speed_motor2 = turn_speed;
		} else {
		set_angular_velocity_motor1(turn_speed);
		set_angular_velocity_motor2(turn_speed);
	}
}

void start_continuous_turn_right(float turn_speed) {
	if (turn_speed < 0) turn_speed = current_turn_speed;
	
	current_motion_state = MOTION_TURN_RIGHT;
	reset_motion_timeout();
	
	set_direction_motor1(0);  // Left wheel forward
	set_direction_motor2(1);  // Right wheel backward
	
	if (smooth_accel_enabled) {
		target_speed_motor1 = turn_speed;
		target_speed_motor2 = turn_speed;
		} else {
		set_angular_velocity_motor1(turn_speed);
		set_angular_velocity_motor2(turn_speed);
	}
}

void stop_all_motors(void) {
	current_motion_state = MOTION_STOPPED;
	motion_timeout_counter = 0;
	
	if (smooth_accel_enabled) {
		target_speed_motor1 = 0.0f;
		target_speed_motor2 = 0.0f;
		} else {
		set_angular_velocity_motor1(0.0f);
		set_angular_velocity_motor2(0.0f);
	}
}

void emergency_stop(void) {
	current_motion_state = MOTION_STOPPED;
	motion_timeout_counter = 0;
	
	// Immediate stop regardless of smooth acceleration setting
	target_speed_motor1 = 0.0f;
	target_speed_motor2 = 0.0f;
	actual_speed_motor1 = 0.0f;
	actual_speed_motor2 = 0.0f;
	
	set_angular_velocity_motor1(0.0f);
	set_angular_velocity_motor2(0.0f);
}

motion_state_t get_current_motion_state(void) {
	return current_motion_state;
}

uint8_t is_motion_active(void) {
	return (current_motion_state != MOTION_STOPPED);
}

void motion_safety_check(void) {
	// This function can be called periodically from main loop
	// Additional safety checks can be added here
	if (motion_timeout_enabled && current_motion_state != MOTION_STOPPED) {
		if (motion_timeout_counter >= motion_timeout_limit) {
			emergency_stop();
		}
	}
}

void reset_motion_timeout(void) {
	motion_timeout_counter = 0;
}

void set_motion_timeout_ms(uint16_t timeout_ms) {
	motion_timeout_limit = timeout_ms;
}

void set_forward_speed(float speed) {
	if (speed > 0) {
		current_forward_speed = speed;
	}
}

void set_turn_speed(float speed) {
	if (speed > 0) {
		current_turn_speed = speed;
	}
}

float get_current_forward_speed(void) {
	return current_forward_speed;
}

float get_current_turn_speed(void) {
	return current_turn_speed;
}

void enable_smooth_acceleration(uint8_t enable) {
	smooth_accel_enabled = enable;
	if (!enable) {
		// If disabling smooth acceleration, apply target speeds immediately
		actual_speed_motor1 = target_speed_motor1;
		actual_speed_motor2 = target_speed_motor2;
		set_angular_velocity_motor1(actual_speed_motor1);
		set_angular_velocity_motor2(actual_speed_motor2);
	}
}

void set_acceleration_rate(float accel_rate) {
	if (accel_rate > 0) {
		acceleration_rate = accel_rate;
	}
}

// Keep all original discrete movement functions for compatibility
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

// max linear velocity = 0.65m/s recommended = 0.55
// max angular velocity = 2.78rad/s  recommended = 2.3




void drive_differential(float vel, float omega) {
	const float GEAR_RATIO = 10.0f;  // 10:1 gear reduction
	
	// Calculate wheel angular velocities
	float w_left_wheel  = (vel + (omega * WHEELBASE / 2.0f)) / WHEEL_RADIUS;
	float w_right_wheel = (vel - (omega * WHEELBASE / 2.0f)) / WHEEL_RADIUS;
	
	// Convert wheel velocities to motor velocities
	// Motor spins 10x faster than wheel due to gear reduction
	float w_left_motor  = w_left_wheel * GEAR_RATIO;
	float w_right_motor = w_right_wheel * GEAR_RATIO;
	
	// Check for maximum motor velocity limits
	float max_motor_velocity = MAX_MOTOR_OMEGA;  // e.g., 104.72 rad/s
	
	if (fabsf(w_left_motor) > max_motor_velocity || fabsf(w_right_motor) > max_motor_velocity) {
		// Scale down both velocities proportionally
		float scale_factor = max_motor_velocity / fmaxf(fabsf(w_left_motor), fabsf(w_right_motor));
		w_left_motor *= scale_factor;
		w_right_motor *= scale_factor;
	}
	
	// Set directions based on wheel velocity signs (not motor velocity signs)
	if (w_left_wheel >= 0) set_direction_motor1(0); else set_direction_motor1(1);
	if (w_right_wheel >= 0) set_direction_motor2(0); else set_direction_motor2(1);
	
	// Set motor angular velocities
	set_angular_velocity_motor1(fabsf(w_left_motor));
	set_angular_velocity_motor2(fabsf(w_right_motor));
}

void move_robot(float linear_vel, float angular_vel, float duration_seconds) {
	// Convert seconds to milliseconds
	uint32_t duration_ms = (uint32_t)(duration_seconds * 1000.0f);
	
	// Start moving with specified velocities
	drive_differential(linear_vel, angular_vel);
	
	// Wait using a loop with fixed 1ms delays
	for (uint32_t i = 0; i < duration_ms; i++) {
		_delay_ms(1);  // Fixed 1ms delay - compiler accepts this
	}
	
	// No stop - robot continues at the last commanded velocity
}

void move_and_stop(float linear_vel, float angular_vel, uint32_t duration_ms) {
	// Start moving with specified velocities
	drive_differential(linear_vel, angular_vel);
	
	// Wait using a loop with fixed 1ms delays
	for (uint32_t i = 0; i < duration_ms; i++) {
		_delay_ms(1);  // Fixed 1ms delay - compiler accepts this
	}
	
	// Stop the robot
	drive_differential(0.0f, 0.0f);
}

void move_ramp_and_stop(float linear_vel, float angular_vel, uint32_t duration_ms) {
	const uint16_t RAMP_TIME_MS = 500;  // 0.5 second ramp
	const uint16_t RAMP_STEPS = 20;     // 20 steps for smooth ramping
	
	// Ramp up
	for (int i = 1; i <= RAMP_STEPS; i++) {
		float scale = (float)i / RAMP_STEPS;
		drive_differential(linear_vel * scale, angular_vel * scale);
		_delay_ms(25);  // Fixed 25ms delay (500ms / 20 steps)
	}
	
	// Move at full speed for remaining time
	drive_differential(linear_vel, angular_vel);
	if (duration_ms > (2 * RAMP_TIME_MS)) {
		uint32_t cruise_time = duration_ms - (2 * RAMP_TIME_MS);
		for (uint32_t i = 0; i < cruise_time; i++) {
			_delay_ms(1);
		}
	}
	
	// Ramp down
	for (int i = RAMP_STEPS; i > 0; i--) {
		float scale = (float)i / RAMP_STEPS;
		drive_differential(linear_vel * scale, angular_vel * scale);
		_delay_ms(25);  // Fixed 25ms delay
	}
	
	// Final stop
	drive_differential(0.0f, 0.0f);
}

// Global variables to track current velocities
volatile float current_linear_vel = 0.0f;
volatile float current_angular_vel = 0.0f;

// Main transition function with linear ramping
void transition_velocity_linear(float new_linear_vel, float new_angular_vel) {
	const uint16_t TRANSITION_STEPS = 50;
	const uint32_t STEP_TIME_MS = 20;  // 1000ms / 50 steps = 20ms per step
	
	// Calculate step increments
	float linear_step = (new_linear_vel - current_linear_vel) / TRANSITION_STEPS;
	float angular_step = (new_angular_vel - current_angular_vel) / TRANSITION_STEPS;
	
	// Smoothly transition step by step
	for (uint16_t i = 1; i <= TRANSITION_STEPS; i++) {
		float temp_linear = current_linear_vel + (linear_step * i);
		float temp_angular = current_angular_vel + (angular_step * i);
		
		drive_differential(temp_linear, temp_angular);
		
		// Fixed 20ms delay (50 steps × 20ms = 1000ms = 1 second)
		for (uint32_t j = 0; j < STEP_TIME_MS; j++) {
			_delay_ms(1);
		}
	}
	
	// Update current velocity tracking
	current_linear_vel = new_linear_vel;
	current_angular_vel = new_angular_vel;
}

// Initialize velocity tracking system
void init_velocity_control(void) {
	current_linear_vel = 0.0f;
	current_angular_vel = 0.0f;
	drive_differential(0.0f, 0.0f);
}

// Get current velocities
void get_current_velocity(float *linear, float *angular) {
	*linear = current_linear_vel;
	*angular = current_angular_vel;
}

// Smooth stop function (1 second stop time)
void smooth_stop(void) {
	transition_velocity_linear(0.0f, 0.0f);
}

// Set velocity instantly (no ramping)
void set_velocity_instant(float linear_vel, float angular_vel) {
	current_linear_vel = linear_vel;
	current_angular_vel = angular_vel;
	drive_differential(linear_vel, angular_vel);
}

// Check if robot is stopped
uint8_t is_robot_stopped(void) {
	return (fabsf(current_linear_vel) < 0.001f && fabsf(current_angular_vel) < 0.001f);
}

// Smooth acceleration to forward motion (1 second acceleration)
void accelerate_forward(float target_velocity) {
	transition_velocity_linear(target_velocity, 0.0f);
}

// Smooth acceleration to turning motion (1 second acceleration)
void accelerate_turn(float target_angular_velocity) {
	transition_velocity_linear(0.0f, target_angular_velocity);
}

// Smooth transition to curve motion (1 second transition)
void accelerate_curve(float linear_vel, float angular_vel) {
	transition_velocity_linear(linear_vel, angular_vel);
}