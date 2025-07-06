

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
	set_direction_motor1(0);
	set_direction_motor2(0);
	//move_distance(0.3,50,1);
	//move_distance(1.83,100,1);
	turn_angle(-M_PI/2,50,1);
	
	while (1) {
	}
}
