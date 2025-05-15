// trapezoid.h
#ifndef TRAPEZOID_H
#define TRAPEZOID_H

#include <avr/io.h>

// Function to accelerate linearly to target angular velocity
void run_trapezoidal_velocity(float target_omega);

#endif