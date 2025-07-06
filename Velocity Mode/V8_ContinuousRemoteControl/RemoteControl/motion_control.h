#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <avr/io.h>

// Movement states for continuous control
typedef enum {
	MOTION_STOPPED = 0,
	MOTION_FORWARD,
	MOTION_BACKWARD,
	MOTION_TURN_LEFT,
	MOTION_TURN_RIGHT
} motion_state_t;




/**
 * Moves both motors a specified linear distance using the selected velocity profile.
 *
 * @param distance_m       Distance to travel in meters
 * @param omega_target     Target angular velocity in rad/s
 * @param use_trapezoid    1 = use trapezoidal profile, 0 = use S-curve profile
 * @param angle_rad        Angle in Radians
 * @param vel              Linear Velocity of AMR in m/s
 * @param omega            Angular Velocity of AMR in rad/s
 */
/**
 * Discrete movement functions
 */
void move_distance(float distance_m, float omega_target, uint8_t use_trapezoid);
float calculate_accel_distance(float omega_target, uint16_t profile_steps);
void turn_angle(float angle_rad, float omega_target, uint8_t use_trapezoid);
void drive_differential(float vel, float omega);

/**
 * Continuous control functions
 */
void motion_control_init(void);
void start_continuous_forward(float speed);
void start_continuous_backward(float speed);
void start_continuous_turn_left(float turn_speed);
void start_continuous_turn_right(float turn_speed);
void stop_all_motors(void);
void emergency_stop(void);

/**
 * State management
 */
motion_state_t get_current_motion_state(void);
uint8_t is_motion_active(void);

/**
 * Safety and timeout functions
 */
void motion_safety_check(void);
void reset_motion_timeout(void);
void set_motion_timeout_ms(uint16_t timeout_ms);

/**
 * Speed control functions
 */
void set_forward_speed(float speed);
void set_turn_speed(float speed);
float get_current_forward_speed(void);
float get_current_turn_speed(void);

/**
 * Smooth acceleration functions (optional for better control)
 */
void enable_smooth_acceleration(uint8_t enable);
void set_acceleration_rate(float accel_rate);

#endif
