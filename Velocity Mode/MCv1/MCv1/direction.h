// direction.h
#ifndef DIRECTION_H
#define DIRECTION_H

#include <avr/io.h>

// Pin definitions
#define STEP_PIN PB5  // OC1A pin (Toggle on compare)
#define DIR_PIN  PB6
#define EN_PIN   PB4

void setup_pins(void);
void set_direction(uint8_t forward);  // 1 = forward, 0 = reverse

#endif

