
// scurve.h
#ifndef SCURVE_H
#define SCURVE_H

#include <avr/io.h>

// Function to accelerate smoothly to target angular velocity using an S-curve
void run_scurve_velocity(float target_omega);

#endif
