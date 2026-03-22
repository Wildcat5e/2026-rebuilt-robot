package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Linux8BitDo extends Xbox {

    /** Uses {@link CommandXboxController} for Simulation Keyboard. @param port index on Driver Station */
    public Linux8BitDo(int port) {
        super(port);
    }

    @Override
    double getRotation() {
        return MathUtil.applyDeadband(-controller.getRawAxis(2), DEADZONE);
    }

    @Override
    Trigger runIntake() {
        return controller.axisGreaterThan(5, 0.5); // Left trigger, threshold from CommandXboxController
    }

    @Override
    Trigger shootFuel() {
        return controller.axisGreaterThan(4, 0.5); // Right trigger, threshold from CommandXboxController

    }

    @Override
    Trigger lowerIntake() {
        return controller.button(8); // Right bumper
    }

    @Override
    Trigger raiseIntake() {
        return controller.button(7); // Left bumper
    }

    @Override
    Trigger seedFieldCentric() {
        return controller.button(4); // X button
    }

    @Override
    Trigger reverse() {
        return controller.button(5); // Y button
    }
}
