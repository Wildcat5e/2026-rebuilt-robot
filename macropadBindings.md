# CircuitPython Macropad Bindings

While the macropad uses 0-indexed buttons, the HID controller we use in [Robot](/src/main/java/frc/robot/Robot.java) is 1-indexed.

* Ensure that your viewer is wide enough to view these bindings tables correctly.

## Base Layer (no modifiers held)

This is the default state of the macropad. All keys run commands continuously while held, except for keys `[1]`, `[2]`, `[4]`, and `[5]`, which only perform their actions once when pressed.

```text
+--------------------+--------------------+--------------------+
| [0]  Bump          | [1]  Flywheel      | [2]  Tunable       |
|     Extender Up    |     Speed Mult +   |  Flywheel Speed +  |
+--------------------+--------------------+--------------------+
| [3]  Bump          | [4]  Flywheel      | [5]  Tunable       |
|     Extender Down  |     Speed Mult -   |  Flywheel Speed -  |
+--------------------+--------------------+--------------------+
| [6]                | [7]  Run Static    | [8]  Run Tunable   |
|                    |      Kicker        |      Flywheel      |
+--------------------+--------------------+--------------------+
| [9]                | [10]               | [11]               |
|    SHIFT KEY       |                    |     CONTROL KEY    |
+--------------------+--------------------+--------------------+
```

## Shift Layer (holding Key 9)

Holding the bottom-left key makes accessible a menu of reverse jogging commands to back fuel out of the robot. All keys run commands continuously while held.

```text
+--------------------+--------------------+--------------------+
| [0]  Scooper       | [1]  Pusher        | [2]  Conveyor      |
|      Reverse       |      Reverse       |      Reverse       |
+--------------------+--------------------+--------------------+
| [3]  Kicker        | [4]  Flywheel      | [5]                |
|      Reverse       |      Reverse       |                    |
+--------------------+--------------------+--------------------+
| [6]                | [7]                | [8]                |
|                    |                    |                    |
+--------------------+--------------------+--------------------+
| [9]                | [10]               | [11]               |
|     [HELD]         |                    |     (Inactive)     |
+--------------------+--------------------+--------------------+
```

## Control Layer (holding Key 11)

Immediately after the bottom-right key is pressed and held, the Kicker begins to spin. While continuing to hold it, the top row acts as three emergency flywheel speeds, to be used in case automated shooting fails. All keys run commands continuously while held.

```text
+--------------------+--------------------+--------------------+
| [0] Spin           | [1] Spin           | [2] Spin           |
|     Flywheel L1    |     Flywheel L2    |     Flywheel L3    |
+--------------------+--------------------+--------------------+
| [3]                | [4]                | [5]                |
|                    |                    |                    |
+--------------------+--------------------+--------------------+
| [6]                | [7]                | [8]                |
|                    |                    |                    |
+--------------------+--------------------+--------------------+
| [9]                | [10]               | [11]               |
|     (Inactive)     |                    |   [HELD] + KICKER  |
+--------------------+--------------------+--------------------+
```
