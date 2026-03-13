"""
Macropad as an FRC auxiliary controller for reverse/jog commands.
Keys 0-4 map to generic HID buttons 1-5.
"""

import board
import usb_hid
from adafruit_macropad import MacroPad
from hid_gamepad import Gamepad

macropad = MacroPad()
gamepad = Gamepad(usb_hid.devices)

RED = 0xff0000
GREEN = 0x00ff00
BLUE = 0x0000ff
YELLOW = 0xffff00
PURPLE = 0xff00ff
BLACK = 0x000000

# Format: (Key Index, Color, "Console Message")
keys = (
    (0, RED, "Scooper Reverse (Button 1)"),
    (1, RED, "Pusher Reverse (Button 2)"),
    (2, RED, "Conveyor Reverse (Button 3)"),
    (3, RED, "Kicker Reverse (Button 4)"),
    (4, RED, "Flywheel Reverse (Button 5)"),
    (5, BLACK, ""),
    (6, BLACK, ""),
    (7, BLACK, ""),
    (8, BLACK, ""),
    (9, BLACK, ""),
    (10, BLACK, ""),
    (11, BLACK, "")
)

while True:
    try:
        if key_event := macropad.keys.events.get():
            if key_event.pressed:
                # Gamepad buttons are 1-indexed
                button_number = key_event.key_number + 1
                gamepad.press_buttons(button_number)
                
                color = keys[key_event.key_number][1]
                message = keys[key_event.key_number][2]
                
                # Illuminate the specific key
                macropad.pixels.fill(BLACK)
                macropad.pixels[key_event.key_number] = color
                if message:
                    print(message)
                    
            elif key_event.released:
                button_number = key_event.key_number + 1
                gamepad.release_buttons(button_number)
                macropad.pixels.fill(BLACK) # Turn off light when released
                
    except Exception as e:
        print(e)