

// main.c (example usage)
#include "config.h"
#include <avr/io.h>
#include "direction.h"
#include "velocity.h"
#include "trapezoid.h"
#include "scurve.h"
#include "velocity.h"
#include <util/delay.h>

#define USE_TRAPEZOID 0 // Set to 1 for trapezoidal, 0 for S-curve

int main(void) {
	setup_pins();
	setup_pins_motor2();

	set_direction(DEFAULT_DIRECTION_M1);
	set_direction_motor2(DEFAULT_DIRECTION_M2);

	ramp_up_trapezoid(DEFAULT_OMEGA_M1);
	//set_angular_velocity(100);
	//set_angular_velocity_motor2(100);
	_delay_ms(2000);
	ramp_down_trapezoid(DEFAULT_OMEGA_M1);
	//set_angular_velocity(0);
	//set_angular_velocity_motor2(0);	
	
	
	while (1) {}
}