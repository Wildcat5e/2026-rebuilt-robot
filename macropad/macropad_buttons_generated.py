# AUTO-GENERATED FILE. DO NOT EDIT.

RED = 0xff0000
GREEN = 0x00ff00
BLUE = 0x0000ff
YELLOW = 0xffff00
PURPLE = 0xff00ff
CYAN = 0x00ffff
BLACK = 0x000000

ACTIONS = [
    "EXTENDER_POS",
    "EXTENDER_NEG",
    "FLYWHEEL_MULT_UP",
    "FLYWHEEL_MULT_DOWN",
    "TUNE_FLYWHEEL_UP",
    "TUNE_FLYWHEEL_DOWN",
    "RUN_HOPPER",
    "TUNABLE_FLYWHEEL",
    "SCOOPER_REV",
    "PUSHER_REV",
    "CONVEYOR_REV",
    "KICKER_REV",
    "FLYWHEEL_REV",
    "SCOOPER_FWD",
    "PUSHER_FWD",
    "CONVEYOR_FWD",
    "HOME_FLYWHEEL_UP",
    "HOME_FLYWHEEL_DOWN",
]

def get_btn(action_name):
    return ACTIONS.index(action_name) + 1

LAYERS = {
    "BASE": {
        0: (get_btn("EXTENDER_POS"), GREEN, "Bump Extender Up"),
        3: (get_btn("EXTENDER_NEG"), GREEN, "Bump Extender Down"),
        1: (get_btn("FLYWHEEL_MULT_UP"), YELLOW, "Flywheel Mult +"),
        4: (get_btn("FLYWHEEL_MULT_DOWN"), YELLOW, "Flywheel Mult -"),
        2: (get_btn("TUNE_FLYWHEEL_UP"), PURPLE, "Tune Flywheel +"),
        5: (get_btn("TUNE_FLYWHEEL_DOWN"), PURPLE, "Tune Flywheel -"),
        7: (get_btn("RUN_HOPPER"), BLUE, "Hopper + Pusher"),
        8: (get_btn("TUNABLE_FLYWHEEL"), BLUE, "Tunable Flywheel")
    },
    "SHIFT": {
        0: (get_btn("SCOOPER_REV"), RED, "Scooper Reverse"),
        1: (get_btn("PUSHER_REV"), RED, "Pusher Reverse"),
        2: (get_btn("CONVEYOR_REV"), RED, "Conveyor Reverse"),
        3: (get_btn("KICKER_REV"), RED, "Kicker Reverse"),
        4: (get_btn("FLYWHEEL_REV"), RED, "Flywheel Reverse"),
        5: (get_btn("HOME_FLYWHEEL_UP"), YELLOW, "Home Flywhl Mult +"),
        8: (get_btn("HOME_FLYWHEEL_DOWN"), YELLOW, "Home Flywhl Mult -")
    },
    "CONTROL": {
        0: (get_btn("SCOOPER_FWD"), PURPLE, "Scooper Forward"),
        1: (get_btn("PUSHER_FWD"), PURPLE, "Pusher Forward"),
        2: (get_btn("CONVEYOR_FWD"), PURPLE, "Conveyor Forward")
    }
}
