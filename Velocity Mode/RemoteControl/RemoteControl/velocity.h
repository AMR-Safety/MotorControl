// velocity.h
#ifndef VELOCITY_H
#define VELOCITY_H

#include <avr/io.h>

void setup_timer1(uint16_t ocr_value);
void setup_timer3(uint16_t ocr_value);

float angular_velocity_to_rpm(float omega);
uint16_t rpm_to_ocr(float rpm);

void set_angular_velocity_motor1(float omega);
void set_angular_velocity_motor2(float omega);

float step_toward_velocity(float current_omega, float target_omega, float step_size);

#endif
