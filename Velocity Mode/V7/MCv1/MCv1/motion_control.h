#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <avr/io.h>

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
void move_distance(float distance_m, float omega_target, uint8_t use_trapezoid);
float calculate_accel_distance(float omega_target, uint16_t profile_steps);
void turn_angle(float angle_rad, float omega_target, uint8_t use_trapezoid);
void drive_differential(float vel, float omega);

#endif
