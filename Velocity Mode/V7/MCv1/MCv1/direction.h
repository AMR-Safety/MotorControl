// direction.h
#ifndef DIRECTION_H
#define DIRECTION_H

#include <avr/io.h>



void setup_pins_motor1(void);
void set_direction_motor1(uint8_t forward);

void setup_pins_motor2(void);
void set_direction_motor2(uint8_t forward);

#endif
