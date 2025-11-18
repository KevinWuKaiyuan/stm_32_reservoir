# Reservoir Pumping System (STM32F401)

This project implements an automated water reservoir control system using an STM32F401RE microcontroller. It monitors water depth, controls multiple pipelines, drives a pump motor using PWM, rotates a servo for pipeline selection, and logs system status over UART.

## Features
- 4-pipeline automated watering system using schedule-based control.
- PWM-driven DC motor with multiple speed modes.
- Servo control (TIM2 PWM) for pipeline selection.
- Ultrasonic depth measurement using HCSR04 (TIM4 Input Capture).
- RPM feedback using an optical or Hall sensor on EXTI.
- Scaled-time scheduling using TIM5 to simulate hour-based operation.
- UART setup interface for configuring pipeline order, pump modes, and schedules.
- Real-time logging of hour, pipeline, PWM mode, RPM, and depth.

## Hardware Requirements
- STM32F401RE (Nucleo-F401RE)
- DC pump motor + MOSFET or driver
- SG90 / MG90 servo for pipe selection
- HCSR04 ultrasonic sensor
- RPM sensor (optical or Hall-based)
- LEDs and 7-segment/bit-style display
- Power supply (5V for motor/servo)

## System Architecture
- TIM2 → Servo PWM
- TIM3 → Pump motor PWM
- TIM4 → Ultrasonic echo timing (Input Capture)
- TIM5 → Scaled timing ("hour" interrupt)
- ADC1 → Speed control in automatic mode
- EXTI2 → RPM tick input
- UART6 → Setup and logging

## Software Flow
### Setup Mode
1. User selects pipeline order.
2. User selects pump speed mode for each pipeline.
3. User enters start and end hours.
4. UART prints the chosen configuration.
5. Blue button proceeds to Run Mode.

### Run Mode
Each simulated hour:
1. Servo rotates to selected pipeline.
2. LED color indicates active pipeline.
3. Motor speed set based on chosen PWM mode.
4. RPM sensor counts ticks during the hour.
5. Ultrasonic sensor measures reservoir depth.
6. System logs: pipeline, PWM, RPM, depth.
7. If depth reaches 0 → system stops and flashes LEDs.

## Repository Structure
```
📦 Reservoir_System
 ┣ 📁 Core
 ┣ 📁 Drivers
 ┣ 📁 docs
 ┣ 📁 media
 ┣ .gitignore
 ┣ LICENSE
 ┗ README.md
```

## How to Build
1. Open project in STM32CubeIDE.
2. Connect the Nucleo-F401RE.
3. Build → Flash via ST-Link.

## Future Improvements
- Closed-loop motor speed control (PID).
- Better RPM calibration.
- Automatic tuning based on depth.
- Web dashboard or SD-card logging.

## License
MIT License. Use freely with attribution.
