# Pre-Match Robot Functionality Checklist

## Mechanical

- Visual Inspection: Give all components a once-over to check for obvious failures (loose bolts, frayed belts, bent metal).

## Electrical

- Battery Health: Check the robot battery using the Battery Beak.

- Does the robot turn on successfully?

- CAN Bus Check: Verify that all 20 devices show up in PhoenixTuner X.

## Programming

- Motor Function: Use Macropad to spin and reverse all motors.

- DriverStation Logs: Check the DriverStation console/log. Verify there are no errors or warnings.

- Vision System Stream: Navigate to <http://photonvision.local:5800/> and verify that both cameras are streaming properly without lag.

- Check the Orange Pi battery level.
    - If the battery is less than 20%, immediately notify the Driver and Operator.
        - You can attach Nicholas's MagSafe battery to charge it while in queue.
        - You cannot run the Orange Pi off of the MagSafe battery as it is not powerful enough. You MUST unplug it before putting the robot on the field.
