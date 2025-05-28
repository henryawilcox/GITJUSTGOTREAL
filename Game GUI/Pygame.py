import pygame
import serial
import struct
import sys
import random
import time

# --- UART Setup ---
PORT = 'COM4'  # Set your STM32 port here
BAUD = 115200
STRUCT_FORMAT = "<BBBBBIHHH"
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

# --- Constants ---
WIDTH, HEIGHT = 800, 500
IR_WIN_THRESHOLD = 3000
DVD_SPEED = [3, 2]

# --- Pygame Setup ---
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("STM32 Game GUI")
font = pygame.font.SysFont("Arial", 28)
clock = pygame.time.Clock()

# --- Sprite Setup ---
sprite_img = pygame.image.load("dvd_logo.png").convert_alpha()
sprite_img = pygame.transform.scale(sprite_img, (100, 50))
sprite_rect = sprite_img.get_rect()
MAX_WIDTH, MAX_HEIGHT = 200, 100  # Or smaller if needed

if sprite_rect.width > MAX_WIDTH or sprite_rect.height > MAX_HEIGHT:
    sprite_img = pygame.transform.scale(sprite_img, (MAX_WIDTH, MAX_HEIGHT))
    sprite_rect = sprite_img.get_rect()

sprite_speed = DVD_SPEED[:]
sprite_rect.topleft = [
    random.randint(0, max(0, WIDTH - sprite_rect.width)),
    random.randint(0, max(0, HEIGHT - sprite_rect.height))
]

# --- Game State ---
state = {
    'start': 0, 'stop': 0, 'reset': 0,
    'level': 1, 'idle': 1, 'time_ms': 0,
    'ir1': 0, 'ir2': 0, 'ir3': 0
}

mode = "menu"  # menu, playing, lost, win
last_result_time = 0

# --- Serial Setup ---
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except serial.SerialException:
    print("Could not open serial port.")
    sys.exit(1)

# --- Drawing Functions ---
def bounce_sprite():
    global sprite_rect, sprite_speed
    sprite_rect.x += sprite_speed[0]
    sprite_rect.y += sprite_speed[1]
    if sprite_rect.left <= 0 or sprite_rect.right >= WIDTH:
        sprite_speed[0] *= -1
    if sprite_rect.top <= 0 or sprite_rect.bottom >= HEIGHT:
        sprite_speed[1] *= -1
    screen.blit(sprite_img, sprite_rect)

def render(text, y, color=(255, 255, 255)):
    label = font.render(text, True, color)
    screen.blit(label, (20, y))

def draw_game(state):
    screen.fill((20, 20, 20))
    render("STM32 Game State", 20)
    render(f"Start:  {'Yes' if state['start'] else 'No'}", 60, (0, 255, 0) if state['start'] else (100, 100, 100))
    render(f"Stop:   {'Yes' if state['stop'] else 'No'}", 90, (255, 0, 0) if state['stop'] else (100, 100, 100))
    render(f"Reset:  {'Yes' if state['reset'] else 'No'}", 120, (255, 255, 0) if state['reset'] else (100, 100, 100))
    render(f"Idle:   {'Yes' if state['idle'] else 'No'}", 150, (0, 200, 255) if state['idle'] else (100, 100, 100))
    render(f"Level:  {state['level']}", 190)
    render(f"Time:   {state['time_ms']} ms", 220)

    # IR Sensor Display
    ir_color = lambda v: (0, 255, 0) if v >= IR_WIN_THRESHOLD else (255, 50, 50)
    render(f"IR1:    {state['ir1']}", 260, ir_color(state['ir1']))
    render(f"IR2:    {state['ir2']}", 290, ir_color(state['ir2']))
    render(f"IR3:    {state['ir3']}", 320, ir_color(state['ir3']))

    bounce_sprite()
    pygame.display.flip()

def draw_menu():
    screen.fill((15, 15, 15))
    title = font.render("SELECT A LEVEL", True, (200, 200, 200))
    screen.blit(title, ((WIDTH - title.get_width()) // 2, 50))

    buttons = []
    for i, text in enumerate(["Level 1", "Level 2", "Level 3"]):
        rect = pygame.Rect(300, 130 + i * 80, 200, 60)
        pygame.draw.rect(screen, (70, 70, 255), rect, border_radius=10)
        label = font.render(text, True, (255, 255, 255))
        screen.blit(label, (rect.x + 50, rect.y + 15))
        buttons.append((rect, i + 1))
    pygame.display.flip()
    return buttons

def draw_result_screen(text, color):
    screen.fill((0, 0, 0))
    label = font.render(text, True, color)
    screen.blit(label, ((WIDTH - label.get_width()) // 2, HEIGHT // 2))
    bounce_sprite()
    pygame.display.flip()

# --- Communication ---
def send_level_to_stm32(level):
    try:
        ser.write(bytes([level]))
    except:
        print("Failed to send level.")

def read_state():
    if ser.in_waiting >= STRUCT_SIZE:
        data = ser.read(STRUCT_SIZE)
        unpacked = struct.unpack(STRUCT_FORMAT, data)
        return {
            'start': unpacked[0],
            'stop': unpacked[1],
            'reset': unpacked[2],
            'level': unpacked[3],
            'idle': unpacked[4],
            'time_ms': unpacked[5],
            'ir1': unpacked[6],
            'ir2': unpacked[7],
            'ir3': unpacked[8],
        }
    return None

# --- Main Loop ---
running = True
level_buttons = []

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if mode == "menu":
            if event.type == pygame.MOUSEBUTTONDOWN:
                for rect, level_val in level_buttons:
                    if rect.collidepoint(event.pos):
                        send_level_to_stm32(level_val)
                        mode = "playing"
                        break

    if mode == "menu":
        level_buttons = draw_menu()

    elif mode == "playing":
        new_state = read_state()
        if new_state:
            state = new_state
            if state['level'] == 0:
                mode = "lost"
                last_result_time = time.time()
            elif state['level'] == 4:
                mode = "win"
                last_result_time = time.time()
        draw_game(state)

    elif mode == "lost":
        draw_result_screen("YOU LOST!", (255, 0, 0))
        if time.time() - last_result_time > 3:
            mode = "menu"

    elif mode == "win":
        draw_result_screen("YOU WIN!", (0, 255, 100))
        if time.time() - last_result_time > 3:
            mode = "menu"

    clock.tick(60)

ser.close()
pygame.quit()


