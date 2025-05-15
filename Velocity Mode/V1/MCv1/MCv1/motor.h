// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <avr/io.h>

// Pin definitions
#define STEP_PIN PB5  // OC1A (hardware toggle pin)
#define DIR_PIN  PB6
#define EN_PIN   PB4

// Function declarations
void setup_pins(void);
void setup_timer1(uint16_t ocr_value);
void set_direction(uint8_t forward);  // 1 = forward, 0 = reverse
uint16_t rpm_to_ocr(float rpm);
float angular_velocity_to_rpm(float omega);
void set_angular_velocity(float omega);




#endif
