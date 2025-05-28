import pygame
import serial
import time
import math

# === SERIAL SETUP ===
# ser = serial.Serial('COM3', 115200)
ser = serial.Serial('COM4', 115200)
time.sleep(2)

# === PYGAME SETUP ===
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Detected controller: {joystick.get_name()}")

# === CONFIG ===
MAX_OUTPUT = 100
DEADZONE = 13
Z_MAX = 11500
Z_STEP = 40
Z_REPEAT_INTERVAL = 0.001  # seconds
SERVO_COMMAND_COOLDOWN = 0.3  # seconds

# === STATE ===
vx_actual = vy_actual = 0
previous_vx = previous_vy = 0
z_target = 0
previous_z_target = -1

last_servo_command_time = 0
last_z_increase_time = 0
last_z_decrease_time = 0

def map_axis(value, deadzone=16, max_output=100):
    raw = int(value * max_output)
    if abs(raw) <= deadzone:
        return 0
    sign = -1 if raw < 0 else 1
    adjusted = abs(raw) - deadzone
    scale_range = max_output - deadzone
    scaled = int(adjusted * max_output / scale_range)
    return sign * max(1, min(scaled, max_output))

def send_servo_command(cmd_byte):
    packet = bytearray(8)
    packet[0] = 0xAA
    packet[1] = cmd_byte
    ser.write(packet)
    print(f"Sent servo command: 0x{cmd_byte:02X}")

# === MAIN LOOP ===
while True:
    pygame.event.pump()
    now = time.time()

    raw_x = joystick.get_axis(0)
    raw_y = joystick.get_axis(1)

    vx_actual = map_axis(-raw_x, DEADZONE, MAX_OUTPUT)
    vy_actual = map_axis(raw_y, DEADZONE, MAX_OUTPUT)

    mag = math.sqrt(vx_actual**2 + vy_actual**2)
    mag_int = int(min(max(mag, 0), 99))

    # === Z-axis control: Triangle = up (3), X = down (0) ===
    triangle_held = joystick.get_button(0)
    x_held = joystick.get_button(3)

    if triangle_held and (now - last_z_increase_time > Z_REPEAT_INTERVAL):
        z_target = min(z_target + Z_STEP, Z_MAX)
        last_z_increase_time = now

    if x_held and (now - last_z_decrease_time > Z_REPEAT_INTERVAL):
        z_target = max(z_target - Z_STEP, 0)
        last_z_decrease_time = now

    # === Send packet if anything changed ===
    if (vx_actual != previous_vx) or (vy_actual != previous_vy) or (z_target != previous_z_target):
        z_lo = z_target & 0xFF
        z_hi = (z_target >> 8) & 0xFF

        packet = bytearray([
            0xAA, 0x01,
            vx_actual & 0xFF,
            vy_actual & 0xFF,
            mag_int & 0xFF,
            z_lo,
            z_hi,
            0x00
        ])

        ser.write(packet)
        print(f"Sent: vx={vx_actual}, vy={vy_actual}, z={z_target}, index={mag_int}")

        previous_vx = vx_actual
        previous_vy = vy_actual
        previous_z_target = z_target

    # === Servo buttons: Square = close (2), Circle = open (1) ===
    square = joystick.get_button(2)
    circle = joystick.get_button(1)

    if square and (now - last_servo_command_time > SERVO_COMMAND_COOLDOWN):
        send_servo_command(0x02)  # Close
        last_servo_command_time = now

    elif circle and (now - last_servo_command_time > SERVO_COMMAND_COOLDOWN):
        send_servo_command(0x03)  # Open
        last_servo_command_time = now

    # === Read back from STM32 ===
    try:
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            if response:
                print(f"Received: {response}")
    except Exception as e:
        print(f"Error reading serial: {e}")

    time.sleep(0.01)
