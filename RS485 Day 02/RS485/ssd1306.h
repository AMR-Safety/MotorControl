#ifndef SSD1306_H
#define SSD1306_H

#include <avr/io.h>
#include <util/delay.h>
#include "twi.h"

#define SSD1306_ADDR 0x3C

void ssd1306_init(void);
void ssd1306_command(uint8_t cmd);
void ssd1306_clear(void);
void ssd1306_display(void);
void ssd1306_set_cursor(uint8_t col, uint8_t row);
void ssd1306_print(const char* str);
void ssd1306_write_char(char c);

#endif
