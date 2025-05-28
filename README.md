# GITJUSTGOTREAL
# MTRX2700 Treasure Hunt Project 🔍

## Overview

This project is an interactive treasure hunt challenge built around three STM32F3 Discovery boards working in tandem. The system is modular and incorporates:
- A **claw mechanism** for object retrieval
- A **controller module** with tilt-based input, lidar for the z axis and a button to open and close the claw
- An **IR sensor array** for detecting if the blocks are placed in the right place.

The system has been designed for modularity with each module working independent.

The STM32 boards communicate using a custom binary serialization protocol over UART, with each board handling a specific subsystem. This README outlines how to set up and integrate the system, as well as a breakdown of each module's functionality.

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
This board handles all claw actuation via stepper motors. It implements:
- Stepper control via timer interrupts
- Position-based or velocity-based movement
- Homing logic with limit switches (deprecated)
- Serial command interface to receive motion commands

**Key files:**
- `stepper.c/h` – Handles motion profiles and step generation
- `command_parser.c/h` – Parses incoming commands and updates state
- `servo.c/h` – Handles the servo motor control

**Inputs:**
- Target velocity from Controller board
- Claw open and close commands

**Outputs:**
- Position reached or movement status

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
