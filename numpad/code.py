"""
Macropad with Shift and Control layers for FRC auxiliary commands.
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
CYAN = 0x00ffff
BLACK = 0x000000

SHIFT_KEY = 9
CTRL_KEY = 11

# Maps physical key index to (HID Button Number, Color, Message)
BASE_LAYER = {
    0: (1, GREEN, "Extender Up 2V"),
    3: (2, GREEN, "Extender Down 1V"),
    1: (3, YELLOW, "Flywheel Mult +"),
    4: (4, YELLOW, "Flywheel Mult -"),
    2: (15, PURPLE, "Stat Flywheel +"),
    5: (16, PURPLE, "Stat Flywheel -"),
    6: (None, BLACK, "\x1b[2J\x1b[H"),
    7: (18, BLUE, "Static Kicker"),
    8: (17, BLUE, "Static Flywheel"),
}

SHIFT_LAYER = {
    0: (5, RED, "Scooper Reverse"),
    1: (6, RED, "Pusher Reverse"),
    2: (7, RED, "Conveyor Reverse"),
    3: (8, RED, "Kicker Reverse"),
    4: (9, RED, "Flywheel Reverse")
    # 10: (10, RED, "E-STOP")
}

CTRL_LAYER = {
    0: (12, PURPLE, "Spin Flywheel L1"),
    1: (13, PURPLE, "Spin Flywheel L2"),
    2: (14, PURPLE, "Spin Flywheel L3")
}

# The HID Button dedicated to spinning the kicker when Control is held
CTRL_KICKER_BUTTON = 11

pressed_buttons = {} # Tracks which layer a button was pressed on to release it correctly
shift_held = False
ctrl_held = False

while True:
    try:
        if key_event := macropad.keys.events.get():
            key = key_event.key_number
            
            if key_event.pressed:
                # Modifier Keys
                if key == SHIFT_KEY:
                    shift_held = True
                    macropad.pixels[SHIFT_KEY] = CYAN
                    print("SHIFT")
                    continue
                elif key == CTRL_KEY:
                    ctrl_held = True
                    macropad.pixels[CTRL_KEY] = CYAN
                    gamepad.press_buttons(CTRL_KICKER_BUTTON) # Spin Kicker immediately
                    print("CONTROL (Kicker)")
                    continue
                
                # Determine which layer dictionary to use
                active_layer = BASE_LAYER
                if ctrl_held:
                    active_layer = CTRL_LAYER
                elif shift_held:
                    active_layer = SHIFT_LAYER
                    
                # Execute mapped command if key exists in the current layer
                if key in active_layer:
                    btn_num, color, message = active_layer[key]

                    if btn_num is not None:
                        gamepad.press_buttons(btn_num)
                        pressed_buttons[key] = btn_num

                    macropad.pixels[key] = color
                    print(message)
                    
            elif key_event.released:
                # Modifier Keys
                if key == SHIFT_KEY:
                    shift_held = False
                    macropad.pixels[SHIFT_KEY] = BLACK
                    continue
                elif key == CTRL_KEY:
                    ctrl_held = False
                    macropad.pixels[CTRL_KEY] = BLACK
                    gamepad.release_buttons(CTRL_KICKER_BUTTON) # Stop Kicker
                    continue
                
                # Release the exact HID button that was pressed
                if key in pressed_buttons:
                    gamepad.release_buttons(pressed_buttons[key])
                    del pressed_buttons[key]
                    macropad.pixels[key] = BLACK
                    
    except Exception as e:
        print(e)