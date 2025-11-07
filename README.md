Key Features

Real-time multitasking using FreeRTOS (separated read / control / protection / display tasks)
Potentiometer-to-PWM control (0–100% setpoint)
Current-based protection with auto-latch and auto-clear behavior
Non-blocking, queue-based inter-task communication
Thread-safe serial logging via a mutex
LED PWM using ESP32 LEDC (20 kHz, 8-bit resolution)
Minimal, production-friendly codebase ready for extension

Hardware

ESP32 Dev Board
Potentiometer connected to GPIO36 (ADC1_CH0)
Current sensor (e.g., ACS712 or equivalent) connected to GPIO39 (ADC1_CH3)
Motor driver with PWM input connected to GPIO18
Motor (as appropriate for your driver)
Power supply sized for motor and ESP32
