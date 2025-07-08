/*
 * RemoteControl.c - Continuous Control Version
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
#include "arrow_parser.h"

int main(void) {
    // Initialize motor hardware
    setup_pins_motor1();
    setup_pins_motor2();
    
    // Set initial directions (both forward)
    set_direction_motor1(0);
    set_direction_motor2(0);
    
	
   
    while (1) {
  
        
    }
    
    return 0;  // Should never reach here
}