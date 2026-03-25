package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Linux8BitDo extends Xbox {
    public Linux8BitDo(int port) {
        super(port);
    }

    @Override public double rotation() {
        return -controller.getRawAxis(2);
    }

    @Override public Trigger runIntake() {
        return controller.axisGreaterThan(5, 0.5); // Left trigger, threshold from CommandXboxController
    }

    @Override public Trigger shootFuel() {
        return controller.axisGreaterThan(4, 0.5); // Right trigger, threshold from CommandXboxController
    }

    @Override public Trigger lowerIntake() {
        return controller.button(8); // Right bumper
    }

    @Override public Trigger raiseIntake() {
        return controller.button(7); // Left bumper
    }

    @Override public Trigger seedFieldCentric() {
        return controller.button(4); // X button
    }

    @Override public Trigger reverse() {
        return controller.button(5); // Y button
    }
}
