#ifndef ARROW_PARSER_H
#define ARROW_PARSER_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

// Command definitions for continuous control
#define CMD_STOP           'S'
#define CMD_FORWARD        'F'
#define CMD_BACKWARD       'B'
#define CMD_TURN_LEFT      'L'
#define CMD_TURN_RIGHT     'R'
#define CMD_HEARTBEAT      'H'   // Keep-alive command

// Response definitions
#define RESP_OK            "OK\n"
#define RESP_MOVING        "MOVING\n"
#define RESP_STOPPED       "STOPPED\n"
#define RESP_ERROR         "ERROR\n"

/**
 * Initialize arrow parser and RS485 communication
 */
void ArrowParser_init(void);

/**
 * Process incoming commands (call continuously in main loop)
 */
void ArrowParser_process_input(void);

/**
 * Send string via RS485 (available for external use)
 */
void RS485_send_str(const char* str);

/**
 * Send single character via RS485
 */
void RS485_send_char(char c);

/**
 * Get last received command
 */
char ArrowParser_get_last_command(void);

/**
 * Check if a new command has been received since last check
 */
uint8_t ArrowParser_has_new_command(void);

/**
 * Clear the new command flag
 */
void ArrowParser_clear_command_flag(void);

/**
 * Enable/disable command echo back to sender
 */
void ArrowParser_set_echo(uint8_t enable);

#endif