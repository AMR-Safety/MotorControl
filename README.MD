# AVR Dual Stepper Motor Control

This project controls two stepper motors using an ATmega32U4-based microcontroller (e.g. Arduino Pro Micro). It features:

- Timer-based pulse generation (Motor 1: Timer1, Motor 2: Timer3)
- Direction and enable control
- Support for trapezoidal and S-curve acceleration profiles
- Modular C file structure for clarity and extensibility

---

## 📦 File Overview

### 🔧 Core Control

- `main.c`  
  Entry point of the program. Selects an acceleration profile and runs the motor at a target angular velocity (in rad/s).

### 🔌 Pin and Direction Control

- `direction.h/.c`  
  Handles setup and direction control for:
  - **Motor 1** (STEP: PB5, DIR: PB6, EN: PB4)
  - **Motor 2** (STEP: PC6, DIR: PB1, EN: PD7)

### 🌀 Velocity and Timer Control

- `velocity.h/.c`  
  Converts angular velocity (rad/s) to timer compare match values using RPM and step rate calculations. Supports:
  - `Timer1` (Motor 1)
  - `Timer3` (Motor 2)

### 📈 Acceleration Profiles

- `trapezoid.h/.c`  
  Implements a **linear ramp** to a target velocity over a fixed number of steps.
  
- `scurve.h/.c`  
  Implements a smooth **S-curve ramp** using a polynomial shaping function.

---

## 🧠 Pin Mapping

| Motor | Signal    | AVR Pin | Arduino Pin | Timer/Feature       |
|--------|-----------|----------|--------------|----------------------|
| 1      | STEP      | PB5      | D9           | Timer1 OC1A (PWM)    |
| 1      | DIR       | PB6      | D10          | Direction control    |
| 1      | EN        | PB4      | D8           | Active LOW enable    |
| 2      | STEP      | PC6      | D5           | Timer3 OC3A (PWM)    |
| 2      | DIR       | PB1      | D15          | Direction control    |
| 2      | EN        | PD7      | D6           | Active LOW enable    |

---

## 🧪 Usage

- V1 - Implemented Both velocity curves for a single motor
- V2 - Implemented Both velocity curves for both single motor
- V3 - Added motion_control which implements moving a given distance according to a given velocity curve
- V4 - Added differential drive and turn angle functions
- V7 - Corrected Move Distance Function
- V8 - Corrected Turn Angle code
- V9 - Corrected Drive differential Code
