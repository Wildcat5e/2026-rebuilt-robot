"""
Note: This code is largely AI-generated and should be thoroughly tested before use.

Macropad with Shift and Control layers for FRC auxiliary commands.
Supports 32 mapped buttons via a custom Gamepad32 class.
Displays current layer and active command on the screen.
"""

import board
import usb_hid
from adafruit_macropad import MacroPad

# Custom Gamepad Class to support 32 buttons (8-byte report)
class Gamepad32:
    
    # Constructor
    def __init__(self, devices):
        for device in devices:
            # Checking hardware IDs to find the specific gamepad device
            if device.usage_page == 0x01 and device.usage == 0x05:
                self._gamepad_device = device
                break
        else:
            # This 'else' belongs to the FOR loop, not the 'if' statement.
            # It only runs if the loop finishes normally without hitting a 'break'.
            raise ValueError("Gamepad device not found")
        
        self._report = bytearray(8)
        self._buttons_state = 0
        self._send()

    # The *buttons syntax allows you to pass any number of button IDs as arguments.
    def press_buttons(self, *buttons):
        for button in buttons:
            # Bitwise operations (|=, <<, etc.) set a specific bit to 1 to record that a button is pressed.
            self._buttons_state |= 1 << (button - 1)
        self._send()

    def release_buttons(self, *buttons):
        for button in buttons:
            # Bitwise AND with NOT mask clears the bit (sets it to 0), releasing the button.
            self._buttons_state &= ~(1 << (button - 1))
        self._send()

    def _send(self):
        # We split the 32-bit integer into 4 separate 8-bit bytes to send over USB.
        self._report[0] = self._buttons_state & 0xFF
        self._report[1] = (self._buttons_state >> 8) & 0xFF
        self._report[2] = (self._buttons_state >> 16) & 0xFF
        self._report[3] = (self._buttons_state >> 24) & 0xFF
        self._gamepad_device.send_report(self._report)

# Now that our Gamepad32 class and constants are defined, we set up our main objects and loop.
macropad = MacroPad()
gamepad = Gamepad32(usb_hid.devices)

# Display Setup
text_lines = macropad.display_text(title="Team 6705 Wildcat5e")

# Defining constants
RED = 0xff0000
GREEN = 0x00ff00
BLUE = 0x0000ff
YELLOW = 0xffff00
PURPLE = 0xff00ff
CYAN = 0x00ffff
BLACK = 0x000000

SHIFT_PHYSICAL_KEY = 9
CTRL_PHYSICAL_KEY = 11

# LAYERS is a Dictionary of Dictionaries. The top-level keys are layer names,
# and the values are dictionaries that map physical key numbers to tuples of
# (gamepad button number, LED color, display message).
LAYERS = {
    "BASE": {
        0: (1, GREEN, "Bump Extender Up"),
        3: (2, GREEN, "Bump Extender Down"),
        1: (3, YELLOW, "Flywheel Mult +"),
        4: (4, YELLOW, "Flywheel Mult -"),
        2: (5, PURPLE, "Tune Flywheel +"),
        5: (6, PURPLE, "Tune Flywheel -"),
        7: (7, BLUE, "Static Kicker"),
        8: (8, BLUE, "Tunable Flywheel"),
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

# Empty dictionary to track which physical keys are currently being held down.
pressed_buttons = {}
current_layer_name = "BASE"

# Python allows default parameters (modifier_msg="", command_msg="").
# If we don't pass in these arguments, they default to empty strings.
def update_display(layer, modifier_msg="", command_msg=""):
    text_lines[0].text = f"Layer: {layer}"
    text_lines[1].text = modifier_msg
    text_lines[2].text = command_msg
    text_lines.show()

# Initial display state
update_display(current_layer_name)

# This is the main program loop.
while True:
    try:
        # The := (walrus operator) assigns the event to 'key_event' AND checks if it's true/exists.
        # It asks: "Did a key just get pressed or released? If so, save it to key_event."
        if key_event := macropad.keys.events.get():
            key = key_event.key_number
            
            # If the physical key was pressed down:
            if key_event.pressed:
                
                # Handle Modifiers (Shift/Ctrl layers)
                if key == SHIFT_PHYSICAL_KEY:
                    current_layer_name = "SHIFT"
                    macropad.pixels[key] = CYAN # Turn the physical LED cyan
                    update_display(current_layer_name)
                
                elif key == CTRL_PHYSICAL_KEY:
                    current_layer_name = "CONTROL"
                    macropad.pixels[key] = CYAN
                    update_display(current_layer_name)
                
                # Handle Standard Keys based on the active layer
                else:
                    active_layer = LAYERS[current_layer_name]
                    
                    # 'in' checks if the key exists in the dictionary
                    if key in active_layer:
                        # This unpacks the tuple into 3 separate variables.
                        btn_num, color, message = active_layer[key]

                        # 'is not None' is a null check
                        if btn_num is not None:
                            gamepad.press_buttons(btn_num)
                            
                        # Add the key to our tracking dictionary
                        pressed_buttons[key] = btn_num 
                        macropad.pixels[key] = color
                        update_display(current_layer_name, text_lines[1].text, message)
                        
                        # Logging to the serial console
                        if btn_num is None:
                            print(f"\x1b[2J\x1b[H--- {message} ---")
                        else:
                            print(message)
                            
            # If the physical key was released:
            elif key_event.released:
                
                # Handle Modifiers
                # 'in (X, Y)' is equivalent to 'if key == X or key == Y'
                if key in (SHIFT_PHYSICAL_KEY, CTRL_PHYSICAL_KEY):
                    current_layer_name = "BASE"
                    macropad.pixels[key] = BLACK # Turns the LED off
                    update_display(current_layer_name)
                
                # Handle Standard Keys
                elif key in pressed_buttons:
                    # .pop() retrieves the value and deletes the key from the dictionary.
                    btn_num = pressed_buttons.pop(key) 
                    
                    if btn_num is not None:
                        gamepad.release_buttons(btn_num)
                        
                    macropad.pixels[key] = BLACK
                    update_display(current_layer_name, text_lines[1].text, "")
                    
    # Standard try-catch block to prevent the main loop from ending on an error.
    except Exception as e:
        print(e)