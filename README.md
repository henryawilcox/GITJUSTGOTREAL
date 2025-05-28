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

-  Ensure system is connected properly with wires to their desination and source correct per diagram
- Use the python GUI to control the operations of the machine
- Pick a level, the higher the level the less time you have
- The button opens and closes the claw while moving your hand infront of the LIDAR sensor picks the height of the claw while moving the STM controller controls the position. How wacky!
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

# LiDAR Module Testing – LED Visualization & Serial Packet Validation

This document outlines how to test the LiDAR sensor module integrated with an STM32F3 microcontroller. The testing is divided into two parts:

1. **LED-Based Distance Visualization**  
2. **Serial Packet Transmission and Python Script Validation**

---

## 🔧 Hardware Setup

- **Microcontroller:** STM32F3
- **Sensor:** LiDAR module (PWM or I2C output)
- **Indicators:** On-board or external LEDs connected to GPIO pins
- **Communication:** UART (e.g., USART1) for serial data transmission

---

## 🧪 1. LED-Based Distance Visualization

### Overview
The LiDAR module outputs distance measurements which are mapped to a range (e.g., 0–100%) and displayed via a set of LEDs.

### How It Works
- The measured height is averaged and converted to a percentage.
- LEDs turn on progressively to represent height/distance bands.

### Example Mapping
- 1 LED ON: 0–25%
- 2 LEDs ON: 25–50%
- 3 LEDs ON: 50–75%
- 4 LEDs ON: 75–100%

### Running the Test
1. Flash your STM32 with the firmware containing the LiDAR + LED logic.
2. Power the board.
3. Move an object at various distances in front of the LiDAR.
4. Observe the number of LEDs lighting up based on distance.

---

## 🧪 2. Serial Packet Transmission & Python Validation

### Overview
LiDAR data is serialized and sent over UART as structured packets (e.g., 8 bytes). A Python script reads and decodes these packets for verification.

### Packet Format (Example)
| Byte Index | Description     |
|------------|------------------|
| 0          | Start Byte (0xAA)|
| 1          | Packet Type      |
| 2          | X-mapped Value   |
| 3          | Y-mapped Value   |
| 4          | Magnitude        |
| 5          | Z low byte       |
| 6          | Z high byte      |
| 7          | End Byte (e.g., checksum or 0xFF) |

### Python Script Requirements
- Python 3.x
- `pyserial` module (`pip install pyserial`)

### Sample Script
```python
import serial
import struct

ser = serial.Serial('COM3', 115200, timeout=1)

def read_packet():
    while True:
        byte = ser.read(1)
        if byte == b'\xAA':
            rest = ser.read(7)
            if len(rest) != 7:
                continue

            packet = byte + rest
            start_byte = packet[0]
            packet_type = packet[1]

            if packet_type == 0x01:
                # Motion packet
                _, _, vx, vy, mag, z_low, z_high, _ = struct.unpack("8B", packet)
                z = z_low | (z_high << 8)
                print(f"[MOTION] vx: {vx}, vy: {vy}, mag: {mag}, z: {z}")

            elif packet_type == 0x02:
                print("[BUTTON] Claw CLOSE (button PRESSED)")

            elif packet_type == 0x03:
                print("[BUTTON] Claw OPEN (button RELEASED)")

            else:
                print(f"[UNKNOWN PACKET] Raw: {packet.hex()}")

try:
    print("Listening for packets...")
    while True:
        read_packet()
except KeyboardInterrupt:
    print("Stopped.")
finally:
    ser.close()

````
## 🧪 Gantry & Stepper Motor Testing (with Controller)

This section outlines how to verify proper operation of the gantry system and its stepper motors using a control interface.

---

### 🔧 Hardware Components

- **Stepper Motors:** X and Y axis (and Z if applicable)
- **Motor Drivers:** e.g., A4988, DRV8825, or TMC series
- **Controller Interface:** Serial terminal, joystick, or button-based UI
- **Microcontroller:** STM32F3 (or equivalent)
- **Power Supply:** Sufficient current for all stepper motors

---

### 🎮 Control Interface Options

1. **Serial Commands via UART**  
   Send motion commands over serial (e.g., `move x +100`, `home y`, etc.)

2. **Joystick or Button Input**  
   Map directional inputs to motor steps for manual movement.

3. **Custom Packet-Based Protocol**  
   Receive structured packets to control movement precisely.

---

### 🧪 Test Procedure

#### ✅ 1. Homing Test
- Send command: `home all`
- Gantry should move to limit switches and stop.
- Verify each axis correctly triggers its limit switch.

#### ✅ 2. Manual Movement Test
- Command or press button to:
  - Move X +10mm → Observe motion.
  - Move Y -10mm → Observe motion.
- Test all directions (X+, X−, Y+, Y−).

#### ✅ 3. Continuous Motion
- Use joystick or a "hold-to-move" button for continuous stepping.
- Check for smooth, uninterrupted motion without skipping steps.

#### ✅ 4. Boundary Testing
- Drive each axis to its max range.
- Verify physical and software limits are respected.
- Ensure no mechanical collisions or overshooting.

#### ✅ 5. Step Calibration (Optional)
- Measure actual travel distance.
- Compare against expected steps/mm.
- Adjust microstepping or step calibration factor if needed.

---

### 🧠 Troubleshooting Tips

- **Motor Not Moving:** Check enable pins, driver wiring, and power supply.
- **Motor Vibrates or Stalls:** Reduce acceleration or step rate.
- **Inconsistent Travel:** Confirm step count per mm and tighten belts or leadscrews.
- **Overtravel:** Re-check limit switch logic and debounce time.

---

### ✅ Expected Outcomes

| Test Step        | Expected Result                                  |
|------------------|--------------------------------------------------|
| Homing           | Axis moves toward limit switches and stops       |
| Manual Move      | Axis moves smoothly in requested direction       |
| Boundaries       | No motion beyond configured travel limits        |
| Continuous Input | Smooth motion with no skipping or jitter         |

---

## 🚀 Ready to Integrate

Once gantry motion and motor control are validated, you can integrate this into your main application loop or automated routines (e.g., pick-and-place, scanning, or claw control).

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
