

// main.c (example usage)
#include "config.h"
#include <avr/io.h>
#include "direction.h"
#include "velocity.h"
#include "trapezoid.h"
#include "scurve.h"
#include "velocity.h"
#include <util/delay.h>

#include "motion_control.h"

int main(void) {
	setup_pins();
	setup_pins_motor2();

	set_direction(DEFAULT_DIRECTION_M1);
	set_direction_motor2(DEFAULT_DIRECTION_M2);

	move_distance(2.0f, DEFAULT_OMEGA_M1, 0);  // Move 0.5 meters using trapezoid (Distance, Omega, Trap=1 , s_curve = 0)

	while (1) {}
}