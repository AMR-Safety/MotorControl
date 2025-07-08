#include "arrow_parser.h"
#include <string.h>
#include "motion_control.h"

#define RE_DE_PIN   PD4
#define RS485_DDR   DDRD
#define RS485_PORT  PORTD
#define BUFFER_SIZE 8

// State variables
static char buffer[BUFFER_SIZE];
static uint8_t idx = 0;
static char last_command = 0;
static uint8_t new_command_flag = 0;
static uint8_t echo_enabled = 1;
static float current_speed = 52.36f;      // Default forward speed
static float current_turn_speed = 50.0f;  // Default turn speed

static void USART1_init(uint16_t baud) {
	uint16_t ubrr = (F_CPU / (16UL * baud)) - 1;
	UBRR1H = (ubrr >> 8);
	UBRR1L = ubrr;
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 8N1
}

static void RS485_send_char_internal(char c) {
	while (!(UCSR1A & (1 << UDRE1)));
	UDR1 = c;
}

void RS485_send_char(char c) {
	RS485_PORT |= (1 << RE_DE_PIN);  // Enable transmit
	_delay_ms(1);
	RS485_send_char_internal(c);
	_delay_ms(1);
	RS485_PORT &= ~(1 << RE_DE_PIN); // Back to receive
}

void RS485_send_str(const char* str) {
	RS485_PORT |= (1 << RE_DE_PIN);  // Enable transmit
	_delay_ms(1);
	while (*str) {
		RS485_send_char_internal(*str++);
	}
	_delay_ms(1);
	RS485_PORT &= ~(1 << RE_DE_PIN); // Back to receive
}

static void send_response(const char* response) {
	if (echo_enabled) {
		RS485_send_str(response);
	}
}

static void execute_command(char cmd) {
	switch (cmd) {
		case CMD_STOP:
		stop_all_motors();
		send_response(RESP_STOPPED);
		break;
		
		case CMD_FORWARD:
		start_continuous_forward(current_speed);
		send_response(RESP_MOVING);
		break;
		
		case CMD_BACKWARD:
		start_continuous_backward(current_speed);
		send_response(RESP_MOVING);
		break;
		
		case CMD_TURN_LEFT:
		start_continuous_turn_left(current_turn_speed);
		send_response(RESP_MOVING);
		break;
		
		case CMD_TURN_RIGHT:
		start_continuous_turn_right(current_turn_speed);
		send_response(RESP_MOVING);
		break;
		
		case CMD_HEARTBEAT:
		// Reset motion timeout on heartbeat
		reset_motion_timeout();
		send_response(RESP_OK);
		break;
		
		default:
		send_response(RESP_ERROR);
		break;
	}
}

static void process_legacy_arrow_keys(void) {
	// Check for legacy escape sequences (ESC + '[' + direction)
	if (idx >= 3 && buffer[idx - 3] == 27 && buffer[idx - 2] == '[') {
		char legacy_cmd = 0;
		
		switch (buffer[idx - 1]) {
			case 'A':  // Up arrow -> Forward
			legacy_cmd = CMD_FORWARD;
			break;
			case 'B':  // Down arrow -> Backward
			legacy_cmd = CMD_BACKWARD;
			break;
			case 'C':  // Right arrow -> Turn right
			legacy_cmd = CMD_TURN_RIGHT;
			break;
			case 'D':  // Left arrow -> Turn left
			legacy_cmd = CMD_TURN_LEFT;
			break;
		}
		
		if (legacy_cmd != 0) {
			last_command = legacy_cmd;
			new_command_flag = 1;
			execute_command(legacy_cmd);
		}
		
		idx = 0;  // Reset buffer
	}
}

static void process_simple_commands(char c) {
	// Check if it's a valid single-character command
	if (c == CMD_STOP || c == CMD_FORWARD || c == CMD_BACKWARD ||
	c == CMD_TURN_LEFT || c == CMD_TURN_RIGHT || c == CMD_HEARTBEAT) {
		
		last_command = c;
		new_command_flag = 1;
		execute_command(c);
		
		idx = 0;  // Reset buffer after processing command
	}
}

void ArrowParser_init(void) {
	RS485_DDR |= (1 << RE_DE_PIN);
	RS485_PORT &= ~(1 << RE_DE_PIN); // Start in receive mode
	USART1_init(9600);
	idx = 0;
	last_command = 0;
	new_command_flag = 0;
	
	// Initialize motion control system
	motion_control_init();
	
	// Send startup message
	send_response("READY\n");
}

void ArrowParser_process_input(void) {
	if (UCSR1A & (1 << RXC1)) {
		char c = UDR1;

		// Store character in buffer for legacy escape sequence processing
		if (idx < BUFFER_SIZE - 1) {
			buffer[idx++] = c;
			buffer[idx] = '\0';
		}

		// Process simple single-character commands first
		process_simple_commands(c);
		
		// Also check for legacy arrow key escape sequences
		process_legacy_arrow_keys();

		// Reset buffer if it gets full to prevent overflow
		if (idx >= BUFFER_SIZE - 1) {
			idx = 0;
		}
	}
	
	// Perform safety check (should be called regularly)
	motion_safety_check();
}

char ArrowParser_get_last_command(void) {
	return last_command;
}

uint8_t ArrowParser_has_new_command(void) {
	return new_command_flag;
}

void ArrowParser_clear_command_flag(void) {
	new_command_flag = 0;
}

void ArrowParser_set_echo(uint8_t enable) {
	echo_enabled = enable;
}