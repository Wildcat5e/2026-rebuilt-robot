"""
Macropad with Shift and Control layers for FRC auxiliary commands.
Supports 32 mapped buttons via a custom Gamepad32 class.
Displays current layer and active command on the screen.
"""

import board
import usb_hid
from adafruit_macropad import MacroPad

# Custom Gamepad Class to support 32 buttons (8-byte report)
class Gamepad32:
    def __init__(self, devices):
        for device in devices:
            if device.usage_page == 0x01 and device.usage == 0x05:
                self._gamepad_device = device
                break
        else:
            raise ValueError("Gamepad device not found")
        
        self._report = bytearray(8)
        self._buttons_state = 0
        self._send()

    def press_buttons(self, *buttons):
        for button in buttons:
            self._buttons_state |= 1 << (button - 1)
        self._send()

    def release_buttons(self, *buttons):
        for button in buttons:
            self._buttons_state &= ~(1 << (button - 1))
        self._send()

    def _send(self):
        self._report[0] = self._buttons_state & 0xFF
        self._report[1] = (self._buttons_state >> 8) & 0xFF
        self._report[2] = (self._buttons_state >> 16) & 0xFF
        self._report[3] = (self._buttons_state >> 24) & 0xFF
        self._gamepad_device.send_report(self._report)

macropad = MacroPad()
gamepad = Gamepad32(usb_hid.devices)

# Set up the OLED screen
text_lines = macropad.display_text(title="Team 6705 Wildcat5e")
text_lines[0].text = "Layer: BASE"
text_lines[1].text = "" # Reserved for Modifier Status (e.g., Kicker Spinning)
text_lines[2].text = "" # Reserved for Active Command
text_lines.show()

RED = 0xff0000
GREEN = 0x00ff00
BLUE = 0x0000ff
YELLOW = 0xffff00
PURPLE = 0xff00ff
CYAN = 0x00ffff
BLACK = 0x000000

SHIFT_KEY = 9
CTRL_KEY = 11

# Maps physical key (0-indexed) to (HID Button Number (1-indexed), Color, Message)
BASE_LAYER = {
    0: (1, GREEN, "Extender Up 2V"),
    3: (2, GREEN, "Extender Down 1V"),
    1: (3, YELLOW, "Flywheel Mult +"),
    4: (4, YELLOW, "Flywheel Mult -"),
    2: (15, PURPLE, "Stat Flywheel +"),
    5: (16, PURPLE, "Stat Flywheel -"),
    7: (18, BLUE, "Static Kicker"),
    8: (17, BLUE, "Static Flywheel"),
}

SHIFT_LAYER = {
    0: (5, RED, "Scooper Reverse"),
    1: (6, RED, "Pusher Reverse"),
    2: (7, RED, "Conveyor Reverse"),
    3: (8, RED, "Kicker Reverse"),
    4: (9, RED, "Flywheel Reverse")
}

CTRL_LAYER = {
    0: (12, PURPLE, "Spin Flywheel L1"),
    1: (13, PURPLE, "Spin Flywheel L2"),
    2: (14, PURPLE, "Spin Flywheel L3")
}

CTRL_KICKER_BUTTON = 11

pressed_buttons = {}
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
                    text_lines[0].text = "Layer: SHIFT"
                    print("SHIFT")
                    continue
                elif key == CTRL_KEY:
                    ctrl_held = True
                    macropad.pixels[CTRL_KEY] = CYAN
                    gamepad.press_buttons(CTRL_KICKER_BUTTON) 
                    text_lines[0].text = "Layer: CONTROL"
                    text_lines[1].text = "Kicker Spinning!"
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
                    text_lines[2].text = message
                    
                    if btn_num is None:
                        # ANSI escape code clears the console
                        print("\x1b[2J\x1b[H", end="")
                        print(f"--- {message} ---")
                    else:
                        print(message)
                    
            elif key_event.released:
                if key == SHIFT_KEY:
                    shift_held = False
                    macropad.pixels[SHIFT_KEY] = BLACK
                    text_lines[0].text = "Layer: BASE"
                    text_lines[1].text = ""
                    continue
                elif key == CTRL_KEY:
                    ctrl_held = False
                    macropad.pixels[CTRL_KEY] = BLACK
                    gamepad.release_buttons(CTRL_KICKER_BUTTON) 
                    text_lines[0].text = "Layer: BASE"
                    text_lines[1].text = ""
                    continue
                
                # Release the button that was pressed
                if key in pressed_buttons:
                    btn_num = pressed_buttons[key]
                    
                    if btn_num is not None:
                        gamepad.release_buttons(btn_num)
                        
                    del pressed_buttons[key]
                    macropad.pixels[key] = BLACK
                    text_lines[2].text = ""
                    
    except Exception as e:
        print(e)