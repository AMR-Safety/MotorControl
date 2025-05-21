#ifndef CONFIG_H
#define CONFIG_H

#define F_CPU 16000000UL

// Stepper motor settings
#define STEPS_PER_REV       800
#define PRESCALER             8

// Trapezoid profile
#define TRAPEZOID_STEPS     50
#define TRAPEZOID_DELAY_MS  50

// S-curve profile
#define SCURVE_STEPS        50
#define SCURVE_DELAY_MS     50

// Target velocities (rad/s)
#define DEFAULT_OMEGA_M1    104.72f  // 1000 RPM
#define DEFAULT_OMEGA_M2    52.36f   // 500 RPM

// Default directions
#define DEFAULT_DIRECTION_M1  1
#define DEFAULT_DIRECTION_M2  0

#endif
