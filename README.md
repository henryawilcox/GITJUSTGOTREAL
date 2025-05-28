# GITJUSTGOTREAL
# MTRX2700 Treasure Hunt Project 🔍

## Overview

This project is an interactive treasure hunt challenge built around three STM32F3 Discovery boards working in tandem. The system is modular and incorporates:
- A **claw mechanism** for object retrieval
- A **controller module** with tilt-based input, lidar for the z axis and a button to open and close the claw
- An **IR sensor array** for detecting if the blocks are placed in the right place.

The system has been designed for modularity with each module working independent.

The STM32 boards communicate using a custom binary serialization protocol over UART, with each board handling a specific subsystem. This README outlines how to set up and integrate the system, as well as a breakdown of each module's functionality.

## Instructions for use

-Ensure system is connected properly with wires to their desination and source correct per diagram
-Use the python GUI to control the operations of the machine
-Pick a level, the higher the level the less time you have
-The button opens and closes the claw while moving your hand infront of the LIDAR sensor picks the height of the claw while moving the STM controller controls the position. How wacky!
*Its that simple**
---

## System Architecture

```
Submodule 1:
[Controller STM32] <-UART-> [Claw STM32]

Submodule 2:
[GUI computer] <-UART-> [IR STM32]
```


Both submodules share a common ground reference. Each board uses a structured packet-based protocol to communicate with the other.

---

## Module Breakdown

### 1. Claw Board (Claw STM32)
This board handles all claw actuation via stepper motors and a servo-controlled claw. It is designed for real-time responsive motion based on velocity commands from the controller board.

#### Stepper Motor Control Design

- A global **40 microsecond interrupt** is triggered by a timer, used to drive stepping decisions across all axes.
- The motion planning is governed by a **velocity index (0–99)** which determines the delay between steps via a `delay_table[]`.
- The system uses **Bresenham’s Line Algorithm** to handle X/Y stepping synchronisation, ensuring smooth diagonal motion.
- **Acceleration and deceleration** are managed using a `timer_reload_table[]`, which ramps the delay values up or down to change speed gradually.
- Each stepper axis maintains its own step delay, position tracking, and direction state.

#### Servo Control

- A command parser handles servo commands (open/close) and updates PWM output.
- Servo angle or duty cycle is set based on parsed command values.
- The claw opens or closes based on a fixed predefined angle or pulse width.

#### Key files:
- `stepper.c/h` – Motion profiles, step logic, ISR
- `command_parser.c/h` – Parses and routes incoming serial commands
- `servo.c/h` – Generates and updates PWM for servo actuation

#### Inputs:
- Target velocity commands (vx, vy) and magnitude index from Controller board
- Claw open/close commands

#### Outputs:
- Position tracking (used internally)
- Optional debug messages or status flags
- Motor enable/disable status for safety

### 2. Controller/Tilt Board (Controller STM32)
This board serves as the input device and human interface. It includes:
- Gyroscope/accelerometer (IMU) for tilt-based control
- Optional button inputs
- Packets containing directional velocity or absolute target positions

**Key functions:**
- Reads and calibrates IMU input
- Converts orientation to movement commands
- Sends velocity commands as packets to the Claw board

**Outputs:**
- `vx`, `vy` values sent as velocity indices (0–99)
- Optional state bits (e.g. button pressed)

### 3. IR Sensor Board (IR STM32)
This board monitors proximity using a bank of IR reflectance sensors. It implements:
- Analog reads via ADC
- Thresholding to detect object presence or wall-following
- Sends discrete detection events or sensor arrays to other boards

**Outputs:**
- Bitfield indicating which IR sensors are triggered
- Optional angle or position offset data

---

## Packet Protocol

All boards communicate via a custom binary packet structure.

### Packet Format

All packets are **8 bytes long** and begin with a single **start byte `0xAA`**. The second byte indicates the **packet type or command**, followed by a payload specific to the message.

There is **no dynamic header or length field**; packets are fixed-size and interpreted based on their second byte.

### Packet Structures

- **Movement Packet**
  - `0xAA` — Start byte
  - `0x01` — Movement command ID
  - `int8_t vx` — X-axis velocity
  - `int8_t vy` — Y-axis velocity
  - `uint8_t mag` — Calculated magnitude (capped at 99)
  - `uint8_t z_low` — LIDAR height (low byte)
  - `uint8_t z_high` — LIDAR height (high byte)
  - `0x00` — Reserved or placeholder for checksum

- **Claw Packet**
  - `0xAA` — Start byte
  - `uint8_t command` — `0x02` = Close, `0x03` = Open
  - `0x00` — Padding
  - `0x00` — Padding
  - `0x00` — Padding
  - `0x00` — Padding
  - `0x00` — Padding
  - `0x00` — Padding
  - 

  -**IR Packet**
- `0x00` - Start bit
- `0x00` - Stop Bit
- `0x00` - Game active bit
- `0x00` - Level state byte
- `0x00` - Time from STM
- `0x00`-Sensor 1 Value
- `0x00` Sensor 2 Value
- `0x00` Sensor 3 Value


### Transmission Order

Packets are sent as **a continuous 8-byte block** with a known format based on the second byte. The receiver reads the first byte (`0xAA`) to confirm the start of a valid packet and then parses based on the packet ID in byte 1.


## Integration Tips

- **Synchronization:** Ensure each STM32 uses the same UART baud rate (typically 115200).
- **Testing:** Each board can be run standalone using test harnesses (e.g. send static packets from a PC).
- **Debugging:** Use serial print or logic analyzers to monitor UART lines during integration.
- **Modularity:** All interface logic is kept in dedicated modules (`serialise.c`, `stepper.c`, etc.) for portability.

---

## Future Improvements

- Full acceleration control (if even possible at real time speeds)
- Introduce handshaking to confirm receipt of critical commands.
- Add GUI-based dashboard for real-time debugging and visualization.

---
