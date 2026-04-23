package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Linux8BitDo extends Xbox {

    // Controller mapping constants
    private static final int ROTATION_AXIS = 2;
    private static final int RIGHT_TRIGGER_AXIS = 4;
    private static final int LEFT_TRIGGER_AXIS = 5;
    private static final int X_BUTTON = 4;
    private static final int Y_BUTTON = 5;
    private static final int LEFT_BUMPER = 7;
    private static final int RIGHT_BUMPER = 8;

    /** Threshold from CommandXboxController. */
    private static final double TRIGGER_THRESHOLD = 0.5;

    /** Uses {@link CommandXboxController} for Linux 8BitDo Controller. @param port index on Driver Station */
    public Linux8BitDo(int port) {
        super(port);
    }

    @Override
    double getRotation() {
        return MathUtil.applyDeadband(-controller.getRawAxis(ROTATION_AXIS), DEADZONE);
    }

    @Override
    Trigger runIntake() {
        return controller.axisGreaterThan(LEFT_TRIGGER_AXIS, TRIGGER_THRESHOLD);
    }

    @Override
    Trigger shootFuel() {
        return controller.axisGreaterThan(RIGHT_TRIGGER_AXIS, TRIGGER_THRESHOLD);
    }

    @Override
    Trigger lowerIntake() {
        return controller.button(RIGHT_BUMPER);
    }

    @Override
    Trigger raiseIntake() {
        return controller.button(LEFT_BUMPER);
    }

    @Override
    Trigger seedFieldCentric() {
        return controller.button(X_BUTTON);
    }

    @Override
    Trigger reverse() {
        return controller.button(Y_BUTTON);
    }
}
