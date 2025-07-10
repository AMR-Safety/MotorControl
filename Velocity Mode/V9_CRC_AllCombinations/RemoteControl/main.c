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
    
    // Initialize communication and motion control systems
    ArrowParser_init();        // This also calls motion_control_init()
    
    // Enable smooth acceleration for comfortable movement
    enable_smooth_acceleration(1);
    //set_acceleration_rate(15.0f);  // Gentle acceleration rate
	set_acceleration_rate(0.042f);  // 41.9 ÷ 1000 = 0.042
    
    // Set default speeds for continuous operation
    set_forward_speed(52.36f);     // Moderate forward speed (rad/s)
    set_turn_speed(40.0f);         // Moderate turn speed (rad/s)
    
    // Set safety timeout to 1 second
    set_motion_timeout_ms(1000);
    
    // Main control loop
    while (1) {
        // Process incoming commands from mini PC
        ArrowParser_process_input();
        
        // Small delay to prevent excessive CPU usage
        // The motion control and safety checks run via interrupts
        _delay_ms(10);
        
    }
    
    return 0;  // Should never reach here
}