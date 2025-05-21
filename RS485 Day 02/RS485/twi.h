#ifndef TWI_H
#define TWI_H

#include <avr/io.h>
#include <stdint.h>

void twi_init(void);
void twi_start(void);
void twi_stop(void);
void twi_write(uint8_t data);

#endif
