/*
 * RemoteControl.c
 *
 * Created: 7/5/2025 8:43:09 AM
 * Author : pasin
 */ 

#define F_CPU 16000000UL

#include "config.h"
#include <avr/io.h>
#include "direction.h"
#include "velocity.h"
#include "trapezoid.h"
#include "scurve.h"
#include <util/delay.h>
#include "motion_control.h"
#include "arrow_parser.h"  // for RS485 input

int main(void) {
	setup_pins_motor1();
	set_direction_motor1(0);
	setup_pins_motor2();
	set_direction_motor2(1);
	//move_distance(10, 104.72, 1);

	ArrowParser_init();

	while (1) {
		ArrowParser_process_input();
	}
}