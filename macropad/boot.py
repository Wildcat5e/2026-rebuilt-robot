import usb_hid

# ----------------------------------------------------------------------------------
# USB HID REPORT DESCRIPTOR
# Think of this as the "blueprint" we hand to the computer.
# It tells the PC exactly what kind of device we are and how to
# decode the raw bytes of data we will send it whenever a button is pressed.
# ----------------------------------------------------------------------------------
GAMEPAD_REPORT_DESCRIPTOR = bytes(
    (
        # --- Device Type Setup ---
        0x05,
        0x01,  # Usage Page: Generic Desktop Controls (we are a desktop device)
        0x09,
        0x05,  # Usage: Gamepad
        0xA1,
        0x01,  # Collection: Application (start grouping our device's features together)
        0x85,
        0x04,  # Report ID: 4 (labels our data as "Device #4" so it doesn't get mixed up with the keyboard or mouse)
        # --- BUTTONS SECTION (32 Buttons) ---
        # We are setting up 32 individual on/off buttons.
        0x05,
        0x09,  # Usage Page: Buttons
        0x19,
        0x01,  # Usage Minimum: Button 1
        0x29,
        0x20,  # Usage Maximum: Button 32
        0x15,
        0x00,  # Logical Minimum: 0 (The value we send when a button is UNPRESSED)
        0x25,
        0x01,  # Logical Maximum: 1 (The value we send when a button is PRESSED)
        0x75,
        0x01,  # Report Size: 1 bit (each button takes up exactly 1 bit of data space)
        0x95,
        0x20,  # Report Count: 32 (we have 32 of these 1-bit buttons)
        0x81,
        0x02,  # Input: Register this data to be sent to the computer
        # --- JOYSTICK AXES SECTION (4 Axes) ---
        # We are setting up 4 directional movements (X, Y, Z, and Rz).
        0x05,
        0x01,  # Usage Page: Generic Desktop Controls (back to joysticks/movements)
        0x15,
        0x81,  # Logical Minimum: -127 (Joystick pushed all the way to one extreme)
        0x25,
        0x7F,  # Logical Maximum: 127 (Joystick pushed all the way to the opposite extreme)
        0x09,
        0x30,  # Usage: X-axis (e.g., Left stick moving Left/Right)
        0x09,
        0x31,  # Usage: Y-axis (e.g., Left stick moving Up/Down)
        0x09,
        0x32,  # Usage: Z-axis (e.g., Right stick moving Left/Right)
        0x09,
        0x35,  # Usage: Rz-axis (e.g., Right stick moving Up/Down)
        0x75,
        0x08,  # Report Size: 8 bits / 1 byte (each axis value uses 1 full byte of space)
        0x95,
        0x04,  # Report Count: 4 (we have 4 of these 1-byte axes)
        0x81,
        0x02,  # Input: Register this data to be sent to the computer
        0xC0,  # End Collection (closes the grouping; our blueprint is complete!)
    )
)

# ----------------------------------------------------------------------------------
# CREATE THE DEVICE
# Now we use the blueprint above to actually construct the virtual device.
# ----------------------------------------------------------------------------------
gamepad = usb_hid.Device(
    report_descriptor=GAMEPAD_REPORT_DESCRIPTOR,
    usage_page=0x01,  # Matches "Generic Desktop Controls" from the blueprint
    usage=0x05,  # Matches "Gamepad" from the blueprint
    report_ids=(4,),  # Matches the Report ID (4) we assigned up top
    in_report_lengths=(
        8,
    ),  # MATH: 32 buttons (4 bytes) + 4 axes (1 byte each) = 8 bytes total sent to the PC
    out_report_lengths=(
        0,
    ),  # We receive 0 bytes from the PC (e.g., we don't have rumble motors or lights to control)
)

# ----------------------------------------------------------------------------------
# ACTIVATE DEVICES
# Tell the microcontroller to turn on these USB features when it connects to the PC.
# ----------------------------------------------------------------------------------
usb_hid.enable(
    (
        usb_hid.Device.KEYBOARD,  # Turn on standard keyboard capability
        usb_hid.Device.MOUSE,  # Turn on standard mouse capability
        usb_hid.Device.CONSUMER_CONTROL,  # Turn on media keys (Volume, Play, Pause, etc.)
        gamepad,  # Turn on our newly created custom gamepad
    )
)
