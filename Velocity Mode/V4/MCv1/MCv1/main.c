

// main.c (example usage)
#include "config.h"
#include <avr/io.h>
#include "direction.h"
#include "velocity.h"
#include "trapezoid.h"
#include "scurve.h"
#include "velocity.h"
#include <util/delay.h>
//#include "modbus_slave.h"

#include "motion_control.h"


int main(void) {
	move_distance(10,104.72,1);
	turn_angle(M_PI / 2.0f, 104.72f, 1);  // 90°, trapezoid
	drive_differential(0.5,0);
	while (1) {
	}
}
