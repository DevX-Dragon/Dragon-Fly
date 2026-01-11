# Dragon Fly Rocket Controller
High-Performance STM32F7-based Flight Controller for Rockets & Drones

<img width="480" height="480" alt="Black Render" src="https://github.com/user-attachments/assets/13e4ca18-dbe3-4623-bb4b-8ea2576e86bd" />
<img width="480" height="480" alt="eye-level" src="https://github.com/user-attachments/assets/c0f371a8-73a0-485c-9b11-516dc6f7455d" />

## Project Overview
This project is a custom, lightweight flight controller designed around the STM32F722RET6 MCU. It features advanced power regulation for wide-voltage battery inputs and high-precision sensing for rocket/drone telemetry.

## Core Specifications
MCU: STM32F722RET6 (216MHz ARM Cortex-M7)

**IMU:** TDK InvenSense ICM-42688-P (High-precision 6-axis)

**Barometer:** BMP580 (Precision altitude sensing)

**Power Supply:** TPS63070 Buck-Boost (Output: 5V @ 2A)

**Logic Voltage:** 3.3V (via LDO)

## Schematics and PCB
<img width="700" height="500" alt="Schematics Layout" src="https://github.com/user-attachments/assets/896a5418-3d3e-4299-bd08-b59250e7b800" />
<img width="300" height="450" alt="PCB Layout" src="https://github.com/user-attachments/assets/7432c427-2b2e-423e-93e3-38333ef0b94a" />


## 🤝 Contributing
Contributions are welcome! If you find a bug in the design or have a suggestion for firmware implementation, please open an Issue or submit a Pull Request.
