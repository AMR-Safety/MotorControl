

// main.c (example usage)
#define F_CPU 16000000UL
#include <avr/io.h>
#include "direction.h"
#include "velocity.h"
#include "trapezoid.h"
#include "scurve.h"

#define USE_TRAPEZOID 0 // Set to 1 for trapezoidal, 0 for S-curve

int main(void) {
	setup_pins();
	set_direction(0);  // Forward

	#if USE_TRAPEZOID
	run_trapezoidal_velocity(104.72f);  // ? 1000 RPM
	#else
	run_scurve_velocity(104.72f);       // ? 1000 RPM
	#endif

	while (1) {
		// Maintain speed
	}
}


