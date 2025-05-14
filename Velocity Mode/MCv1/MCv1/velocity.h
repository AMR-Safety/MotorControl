// velocity.h
#ifndef VELOCITY_H
#define VELOCITY_H

#include <avr/io.h>

void setup_timer1(uint16_t ocr_value);
float angular_velocity_to_rpm(float omega);
uint16_t rpm_to_ocr(float rpm);
void set_angular_velocity(float omega);

#endif

