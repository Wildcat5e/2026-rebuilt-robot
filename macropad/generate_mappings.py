import json
import os


def generate_files():
    # 1. Setup file paths automatically based on the script's location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(script_dir, "macropad-layout.json")
    py_out_path = os.path.join(script_dir, "macropad_buttons_generated.py")

    # Trace the path back out of 'macropad' and into the java source directory
    java_out_dir = os.path.join(
        script_dir,
        "..",
        "src",
        "main",
        "java",
        "frc",
        "robot",
        "controller",
        "operator",
    )
    java_out_path = os.path.join(java_out_dir, "MacropadButtonsGenerated.java")

    if not os.path.exists(json_path):
        print(f"Error: Could not find {json_path}")
        return

    with open(json_path, "r") as f:
        layout = json.load(f)

    # -----------------------------------------
    # 2. Generate the Java Enum
    # -----------------------------------------
    os.makedirs(
        java_out_dir, exist_ok=True
    )  # Ensure the java directory actually exists

    java_code = "package frc.robot.controller.operator;\n\n"
    java_code += "// AUTO-GENERATED FILE. DO NOT EDIT.\n"
    java_code += "public enum MacropadButtonsGenerated {\n"

    for i, btn in enumerate(layout):
        action = btn["action"]
        if i < len(layout) - 1:
            java_code += f"    {action},\n"
        else:
            java_code += f"    {action}\n"

    java_code += "}\n"

    with open(java_out_path, "w") as f:
        f.write(java_code)

    # -----------------------------------------
    # 3. Generate the CircuitPython Layers
    # -----------------------------------------
    py_code = "# AUTO-GENERATED FILE. DO NOT EDIT.\n\n"

    # Define colors here so code.py can be perfectly clean
    py_code += "RED = 0xff0000\n"
    py_code += "GREEN = 0x00ff00\n"
    py_code += "BLUE = 0x0000ff\n"
    py_code += "YELLOW = 0xffff00\n"
    py_code += "PURPLE = 0xff00ff\n"
    py_code += "CYAN = 0x00ffff\n"
    py_code += "BLACK = 0x000000\n\n"

    # Generate the ACTIONS list to keep order synced
    py_code += "ACTIONS = [\n"
    for btn in layout:
        py_code += f'    "{btn["action"]}",\n'
    py_code += "]\n\n"

    py_code += "def get_btn(action_name):\n"
    py_code += "    return ACTIONS.index(action_name) + 1\n\n"

    # Group the buttons by their assigned Layer
    layers = {}
    for btn in layout:
        layer = btn["layer"]
        if layer not in layers:
            layers[layer] = []
        layers[layer].append(btn)

    # Generate the LAYERS dictionary
    py_code += "LAYERS = {\n"
    layer_items = list(layers.items())

    for i, (layer_name, btns) in enumerate(layer_items):
        py_code += f'    "{layer_name}": {{\n'

        for j, btn in enumerate(btns):
            action = btn["action"]
            key = btn["key"]
            color = btn["color"]
            text = btn["text"]

            comma = "," if j < len(btns) - 1 else ""
            py_code += (
                f'        {key}: (get_btn("{action}"), {color}, "{text}"){comma}\n'
            )

        comma_layer = "," if i < len(layer_items) - 1 else ""
        py_code += f"    }}{comma_layer}\n"

    py_code += "}\n"

    with open(py_out_path, "w") as f:
        f.write(py_code)

    print("Successfully generated MacropadButtonsGenerated.java and macropad_buttons_generated.py.")


if __name__ == "__main__":
    generate_files()
