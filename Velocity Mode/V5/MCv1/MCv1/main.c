

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
	setup_pins_motor1();
	setup_pins_motor2();
	move_distance(10,104.72,1);
	while (1) {
	}
}
