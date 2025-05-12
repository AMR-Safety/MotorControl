#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#define STEP_PIN PB6
#define DIR_PIN  PB5
#define EN_PIN   PB4

#define MAX_OCR 300     // Fastest stepping rate
#define MIN_OCR 3000    // Slowest stepping rate
#define STEPS_PER_MM 25 // Adjust based on your hardware

#define RAMP_FRAC 0.2   // 20% ramp up, 60% coast, 20% ramp down

volatile uint32_t step_count = 0;
volatile uint32_t total_steps = 0;
volatile uint32_t ramp_steps = 0;
volatile uint32_t coast_start = 0;
volatile uint32_t coast_end = 0;
volatile uint16_t current_ocr = MIN_OCR;
volatile uint8_t motion_done = 0;

void setup_pins() {
    DDRB |= (1 << STEP_PIN) | (1 << DIR_PIN) | (1 << EN_PIN);
    PORTB &= ~(1 << EN_PIN); // Enable driver (active low)
    PORTB |= (1 << DIR_PIN); // Forward direction
}

void setup_timer1(uint16_t ocr) {
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11); // CTC mode, prescaler 8
    OCR1A = ocr;
    TIMSK1 |= (1 << OCIE1A);             // Enable compare interrupt
    sei();
}

// S-curve function: 3x² - 2x³
float s_curve(float x) {
    return 3 * x * x - 2 * x * x * x;
}

// Compute OCR based on phase
uint16_t compute_ocr(uint32_t step) {
    if (step < ramp_steps) {
        // Acceleration
        float x = (float)step / ramp_steps;
        float factor = s_curve(x);
        return (uint16_t)(MIN_OCR - factor * (MIN_OCR - MAX_OCR));
    } else if (step < coast_end) {
        // Constant speed
        return MAX_OCR;
    } else if (step < total_steps) {
        // Deceleration
        float x = (float)(step - coast_end) / ramp_steps;
        float factor = s_curve(1 - x);
        return (uint16_t)(MIN_OCR - factor * (MIN_OCR - MAX_OCR));
    } else {
        return MAX_OCR;
    }
}

ISR(TIMER1_COMPA_vect) {
    static uint8_t toggle = 0;

    if (motion_done) {
        TCCR1B = 0;
        return;
    }

    if (toggle) {
        PORTB &= ~(1 << STEP_PIN);
        toggle = 0;
    } else {
        PORTB |= (1 << STEP_PIN);
        toggle = 1;

        step_count++;
        if (step_count < total_steps) {
            current_ocr = compute_ocr(step_count);
            OCR1A = current_ocr;
        } else {
            motion_done = 1;
        }
    }
}

// Move a given distance in mm
void move_distance_mm(float distance_mm) {
    step_count = 0;
    motion_done = 0;
    total_steps = (uint32_t)(distance_mm * STEPS_PER_MM);

    ramp_steps = (uint32_t)(total_steps * RAMP_FRAC);
    coast_start = ramp_steps;
    coast_end = total_steps - ramp_steps;

    current_ocr = MIN_OCR;
    setup_timer1(current_ocr);
}

int main(void) {
    setup_pins();

    float target_distance = 100.0; // Change as needed
    move_distance_mm(target_distance);

    while (1) {
        if (motion_done) {
            PORTB |= (1 << EN_PIN); // Disable driver
            while (1); // Hold
        }
    }
}