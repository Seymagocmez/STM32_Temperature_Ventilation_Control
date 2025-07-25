# Temperature-Controlled Ventilation System for Indoor Environments

This STM32-based system provides precise temperature-controlled ventilation, featuring automatic and manual fan control, real-time monitoring, and comprehensive alert mechanisms.


[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Project Status: Completed](https://img.shields.io/badge/Status-Completed-brightgreen.svg)

## Table of Contents

* [ Collaborators](#-collaborators)
* [Project Overview](#-project-overview)
* [Features](#-features)
    * [Temperature Measurement](#temperature-measurement)
    * [Fan Control](#fan-control)
    * [Motor Control](#motor-control)
    * [Alerts](#alerts)
    * [Communication](#communication)
    * [Interrupt Handling](#interrupt-handling)
    * [Custom LCD Library](#custom-lcd-library)
* [Hardware Components](#-hardware-components)
* [System Logic](#-system-logic)
    * [Digital Control Flow](#digital-control-flow)
    * [Fan Direction](#fan-direction)
* [Implementation](#-implementation)
    * [Tools & Environment](#tools--environment)
    * [STM32CubeMX Pinout](#stm32cubemx-pinout)
    * [Supplementary Configuration Tables](#supplementary-configuration-tables)
    * [Proteus Simulation Setup](#proteus-simulation-setup)
* [How to Run](-how-to-run)
    * [Run in Proteus (Simulation)](#run-in-proteus-simulation)
* [Documentation](#documentation)
* [License](#-license)


## 🤝 Collaborators

This project was developed through the valuable collaboration of:

[![Merve Elanur Azman](https://img.shields.io/badge/Merve%20Elanur%20Azman-Collaborator-blue?style=for-the-badge&logo=github)](https://github.com/MerveElanurAzman)
[![Gül Nur Deviren](https://img.shields.io/badge/G%C3%BCl%20Nur%20Deviren-Collaborator-green?style=for-the-badge&logo=github)](https://github.com/gulnurdeviren)
[![Şeyma Göçmez](https://img.shields.io/badge/%C5%9Eeyma%20G%C3%B6%C3%A7mez-Collaborator-purple?style=for-the-badge&logo=github)](https://github.com/Seymagocmez)


## Project Overview

The system controls indoor temperature using an **LM35 sensor** and displays real-time temperature on an **LCD1602 display**. An electric **DC fan** can be controlled automatically or manually:

* **Automatic Mode:** The fan adjusts direction and speed based on predefined temperature thresholds.
* **Manual Mode:** The user controls fan direction (forward/reverse) and speed using push buttons.

**Visual (LEDs)** and **audible (buzzer)** alerts notify users when the temperature is outside the safe range.
## Features
### Temperature Measurement

* **LM35 analog sensor**
* Real-time display on **LCD1602**
* Updates triggered by ≥0.5°C change

### Fan Control

* **Auto Mode:**

    * Above 20°C → Counter-clockwise (cooling mode)
    * Below 10°C → Clockwise (heating mode)
    * Between 10–20°C → Fan stops

* **Manual Mode:**

    * 5 push buttons control fan direction (FWD, REV, STOP)
    * PWM speed control (`+10%` / `-10%` per button press)

### Motor Control

* **L298N Dual H-Bridge** for direction control
* **PWM-based speed control (0–100%)**

### Alerts

* **Red LED + Buzzer:** Danger zone (Temp <10°C or >20°C)
* **Green LED:** Safe range (10–20°C)

### Communication

* **UART (Serial Output):** Transmits PWM duty cycle for monitoring

### Interrupt Handling

* **EXTI interrupts** handle button inputs for real-time control

### Custom LCD Library

* `lcd1602.h` driver written from scratch using STM32 HAL GPIO functions

---

## Hardware Components

| Component           | Description                                  |
| :------------------ | :------------------------------------------- |
| **MCU** | STM32F103R6                              |
| **Sensor** | LM35 Analog Temperature Sensor               |
| **Motor Driver** | L298N Dual H-Bridge                          |
| **Display** | LCD1602 (16x2 Character Display)             |
| **Fan** | DC Motor with adjustable direction/speed     |
| **Push Buttons** | 5 (FWD, REV, STOP, PWM+, PWM-)               |
| **LEDs + Buzzer** | Red & Green LEDs + Buzzer for alerts         |
| **UART** | Virtual terminal output for PWM monitoring   |

---

##  System Logic

### Digital Control Flow

<div align="center">

<img width="1920" height="1080" alt="Control Flow Diagram" src="https://github.com/user-attachments/assets/8121153a-79c8-48fc-985b-5be50f3f2f88" />

<br>

*Figure 1: Ventilation System Digital Connections and Control Logic.*

</div>

#### Workflow:

* **Initialize hardware**
* **Loop:**

    * Read LM35 sensor
    * Update LCD if temp changes ≥0.5°C
    * Check mode:

        * **Manual Mode:** Set direction & speed via buttons
        * **Auto Mode:**

            * > 20°C → Fan CCW + Alerts
            * <10°C → Fan CW + Alerts
            * 10–20°C → Fan stops, green LED on

### Fan Direction

<div align="center">

<img width="948" height="361" alt="Fan Direction Diagram" src="https://github.com/user-attachments/assets/64aec744-795f-4cf4-a03d-1bcf63892bd3" />

<br>

*Figure 2: Fan Direction Control Mechanism.*

</div>

| Mode        | Direction         | Purpose              | GPIO              |
| :---------- | :---------------- | :------------------- | :---------------- |
| Summer Mode | Counter-clockwise | Cooling airflow      | PC8=HIGH, PC9=LOW |
| Winter Mode | Clockwise         | Warm air circulation | PC8=LOW, PC9=HIGH |

---

##  Implementation

### Tools & Environment

* **STM32CubeMX:** Peripheral configuration
* **Keil MDK-ARM:** Embedded C programming
* **Proteus 8.17:** Circuit simulation

### STM32CubeMX Pinout

<div align="center">

<img width="627" height="583" alt="STM32CubeMX Pinout" src="https://github.com/user-attachments/assets/29106599-ca95-4623-93d6-d3fd8cc1a78e" />

<br>

*Figure 3: STM32CubeMX Pinout Configuration.*

</div>

### Supplementary Configuration Tables

<div align="center">

<img width="885" height="615" alt="Supplementary Config Tables" src="https://github.com/user-attachments/assets/3cea04f6-8d4b-461f-bfd6-bb6a00209f09" />

<br>

*Figure 4: Supplementary Configuration Tables.*

</div>

### Proteus Simulation Setup

#### Connections:

| Function    | STM32 Pin       |
| :---------- | :-------------- |
| LM35 Sensor | PA1 (ADC)       |
| LCD Control | PA2–PA4         |
| LCD Data    | PC0–PC6, PC10   |
| Fan PWM     | PC7 (TIM3_CH2)  |
| Fan Dir     | PC8, PC9        |
| UART TX     | PA9             |
| Red LED     | PB11            |
| Green LED   | PB10            |
| Buzzer      | PA0             |
| Buttons     | PB0–PB4         |

#### Simulation Snapshots:

<table>
  <tr>
    <td align="center">
      <img width="934" height="725" alt="Safe Temp" src="https://github.com/user-attachments/assets/d728470d-846d-4eff-ace1-d076b31d4423" />
      <br>
      <b>Safe Temp</b>
    </td>
    <td align="center">
      <img width="972" height="744" alt="High Temp" src="https://github.com/user-attachments/assets/ef96268f-b2ab-459f-a96b-cdaa7d1e2ca0" />
      <br>
      <b>High Temp</b>
    </td>
  </tr>
</table>

---

## How to Run

### Run in Proteus (Simulation)

#### Prerequisites

- **Proteus 8.17+**
- **Keil MDK-ARM 5+**
- **STM32CubeIDE (Optional)**

#### Steps

```bash
git clone https://github.com/Seymagocmez/STM32_Temperature_Ventilation_Control.git
cd STM32_Temperature_Ventilation_Control
```

1. Open the `ventilition_system.pdsprj` file in Proteus.
2. Open the Keil project (`ventilition_system.uvprojx`) in Keil MDK-ARM.
3. Build the project in Keil to generate the `.hex` file.
4. Load the generated `.hex` file into the STM32 MCU component in the Proteus schematic.
5. Start the simulation in Proteus to observe the system behavior.

For a more detailed documentation, please refer to the [project docs](https://github.com/Seymagocmez/STM32_Temperature_Ventilation_Control/blob/main/STM32F103R6_VENTILATION_CONTROL_DOCUMENTATION.pdf).

## Documentation

For a more detailed explanation of the system design, implementation details, and usage instructions, please refer to the [project documentation (PDF)](https://github.com/Seymagocmez/STM32_Temperature_Ventilation_Control/blob/main/STM32F103R6_VENTILATION_CONTROL_DOCUMENTATION.pdf).


## License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).  
See the [LICENSE](./LICENSE) file for details.
