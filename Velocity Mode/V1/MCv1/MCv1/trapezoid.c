// trapezoid.c
#include "trapezoid.h"
#include "velocity.h"
#define F_CPU 16000000UL
#include <util/delay.h>

// Configure this based on desired acceleration smoothness
#define STEPS 50
#define ACCEL_DELAY_MS 50

void run_trapezoidal_velocity(float target_omega) {
	for (int i = 1; i <= STEPS; i++) {
		float omega = (target_omega * i) / STEPS;  // Linear ramp
		set_angular_velocity(omega);
		_delay_ms(ACCEL_DELAY_MS);
	}
	// Now maintain final velocity
	set_angular_velocity(target_omega);
}

