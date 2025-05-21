// direction.h
#ifndef DIRECTION_H
#define DIRECTION_H

#include <avr/io.h>

// Motor 1 Pins
#define STEP_PIN       PB5 //D9
#define DIR_PIN        PB6 //D10
#define EN_PIN         PB4 //D8

// Motor 2 Pins
#define STEP2_PIN      PC6 //D5
#define DIR2_PIN       PB1 //D15
#define EN2_PIN        PD7 //D6


void setup_pins(void);
void set_direction(uint8_t forward);

void setup_pins_motor2(void);
void set_direction_motor2(uint8_t forward);

#endif
