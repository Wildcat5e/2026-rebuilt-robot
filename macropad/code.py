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

# Display Setup
text_lines = macropad.display_text(title="Team 6705 Wildcat5e")

RED = 0xff0000
GREEN = 0x00ff00
BLUE = 0x0000ff
YELLOW = 0xffff00
PURPLE = 0xff00ff
CYAN = 0x00ffff
BLACK = 0x000000

SHIFT_KEY = 9
CTRL_KEY = 11
CTRL_KICKER_BUTTON = 11

# Layer Dictionaries
LAYERS = {
    "BASE": {
        0: (1, GREEN, "Extender Up 2V"),
        3: (2, GREEN, "Extender Down 1V"),
        1: (3, YELLOW, "Flywheel Mult +"),
        4: (4, YELLOW, "Flywheel Mult -"),
        2: (5, PURPLE, "Stat Flywheel +"),
        5: (6, PURPLE, "Stat Flywheel -"),
        7: (7, BLUE, "Static Kicker"),
        8: (8, BLUE, "Static Flywheel"),
    },
    "SHIFT": {
        0: (9, RED, "Scooper Reverse"),
        1: (10, RED, "Pusher Reverse"),
        2: (11, RED, "Conveyor Reverse"),
        3: (12, RED, "Kicker Reverse"),
        4: (13, RED, "Flywheel Reverse")
    },
    "CONTROL": {
        0: (14, PURPLE, "Spin Flywheel L1"),
        1: (15, PURPLE, "Spin Flywheel L2"),
        2: (16, PURPLE, "Spin Flywheel L3")
    }
}

pressed_buttons = {}
current_layer_name = "BASE"

def update_display(layer, modifier_msg="", command_msg=""):
    text_lines[0].text = f"Layer: {layer}"
    text_lines[1].text = modifier_msg
    text_lines[2].text = command_msg
    text_lines.show()

# Initial display state
update_display(current_layer_name)

while True:
    try:
        if key_event := macropad.keys.events.get():
            key = key_event.key_number
            
            if key_event.pressed:
                # Handle Modifiers
                if key == SHIFT_KEY:
                    current_layer_name = "SHIFT"
                    macropad.pixels[key] = CYAN
                    update_display(current_layer_name)
                
                elif key == CTRL_KEY:
                    current_layer_name = "CONTROL"
                    macropad.pixels[key] = CYAN
                    gamepad.press_buttons(CTRL_KICKER_BUTTON) 
                    update_display(current_layer_name, modifier_msg="Kicker Spinning!")
                
                # Handle Standard Keys
                else:
                    active_layer = LAYERS[current_layer_name]
                    if key in active_layer:
                        btn_num, color, message = active_layer[key]

                        if btn_num is not None:
                            gamepad.press_buttons(btn_num)
                            
                        pressed_buttons[key] = btn_num 
                        macropad.pixels[key] = color
                        update_display(current_layer_name, text_lines[1].text, message)
                        
                        if btn_num is None:
                            print(f"\x1b[2J\x1b[H--- {message} ---")
                        else:
                            print(message)
                    
            elif key_event.released:
                # Handle Modifiers
                if key in (SHIFT_KEY, CTRL_KEY):
                    current_layer_name = "BASE"
                    macropad.pixels[key] = BLACK
                    
                    if key == CTRL_KEY:
                        gamepad.release_buttons(CTRL_KICKER_BUTTON) 
                        
                    update_display(current_layer_name)
                
                # Handle Standard Keys
                elif key in pressed_buttons:
                    btn_num = pressed_buttons.pop(key) # Retrieves and deletes in one step
                    
                    if btn_num is not None:
                        gamepad.release_buttons(btn_num)
                        
                    macropad.pixels[key] = BLACK
                    update_display(current_layer_name, text_lines[1].text, "")
                    
    except Exception as e:
        print(e)