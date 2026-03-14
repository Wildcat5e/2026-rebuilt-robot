# CircuitPython Numberpad Bindings

While the numberpad uses 0-indexed buttons, the HID controller we use in [Robot](/src/main/java/frc/robot/Robot.java) is 1-indexed.

* Ensure that your viewer is wide enough to view these bindings tables correctly.

## Base Layer (no modifiers held)

This is the default state of the numberpad.

```text
+--------------------+--------------------+--------------------+
| [0] Extender       | [1] Flywheel       | [2]                |
|     Up             |     Mult +         |       (None)       |
+--------------------+--------------------+--------------------+
| [3] Extender       | [4] Flywheel       | [5]                |
|     Down           |     Mult -         |       (None)       |
+--------------------+--------------------+--------------------+
| [6]                | [7] Clear          | [8]                |
|       (None)       |     Terminal       |       (None)       |
+--------------------+--------------------+--------------------+
| [9]                | [10]               | [11]               |
|     SHIFT KEY      |       (None)       |     CONTROL KEY    |
+--------------------+--------------------+--------------------+
```

## Shift Layer (holding Key 9)

Holding the bottom-left key activates reverse jogging commands to back fuel out of the robot, as well as the software E-Stop.

```text
+--------------------+--------------------+--------------------+
| [0] Scooper        | [1] Pusher         | [2] Conveyor       |
|     Reverse        |     Reverse        |     Reverse        |
+--------------------+--------------------+--------------------+
| [3] Kicker         | [4] Flywheel       | [5]                |
|     Reverse        |     Reverse        |       (None)       |
+--------------------+--------------------+--------------------+
| [6]                | [7]                | [8]                |
|       (None)       |       (None)       |       (None)       |
+--------------------+--------------------+--------------------+
| [9]                | [10]               | [11]               |
|     [HELD]         |     E-STOP         |     (Inactive)     |
+--------------------+--------------------+--------------------+
```

## Control Layer (holding Key 11)

The moment the bottom-right key is pressed and held, the Kicker begins to spin immediately. While continuing to hold it, the top row acts as three emergency flywheel speeds, to be used in case automated shooting fails.

```text
+--------------------+--------------------+--------------------+
| [0] Spin           | [1] Spin           | [2] Spin           |
|     Flywheel L1    |     Flywheel L2    |     Flywheel L3    |
+--------------------+--------------------+--------------------+
| [3]                | [4]                | [5]                |
|       (None)       |       (None)       |       (None)       |
+--------------------+--------------------+--------------------+
| [6]                | [7]                | [8]                |
|       (None)       |       (None)       |       (None)       |
+--------------------+--------------------+--------------------+
| [9]                | [10]               | [11]               |
|     (Inactive)     |       (None)       |  [HELD] + KICKER   |
+--------------------+--------------------+--------------------+
```
