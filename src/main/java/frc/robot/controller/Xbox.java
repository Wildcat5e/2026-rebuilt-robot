package frc.robot.controller;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Xbox implements Controller {
    protected final CommandXboxController controller;

    public Xbox(int port) {
        controller = new CommandXboxController(port);
    }

    @Override public Translation2d translation() {
        return new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    }

    @Override public double rotation() {return -controller.getRightX();}

    @Override public Trigger runIntake() {
        return controller.leftTrigger();
    }

    @Override public Trigger shootFuel() {
        return controller.rightTrigger();
    }

    @Override public Trigger lowerIntake() {
        return controller.rightBumper();
    }

    @Override public Trigger raiseIntake() {
        return controller.leftBumper();
    }

    @Override public Trigger aimHandler() {
        return controller.a();
    }

    @Override public Trigger manualFlywheel() {
        return controller.b();
    }

    @Override public Trigger seedFieldCentric() {
        return controller.x();
    }

    @Override public Trigger reverse() {
        return controller.y();
    }

    @Override public Trigger povUp() {
        return controller.povUp();
    }

    @Override public Trigger povDown() {
        return controller.povDown();
    }

    @Override public Trigger povLeft() {
        return controller.povLeft();
    }

    @Override public Trigger povRight() {
        return controller.povRight();
    }

    @Override public Trigger forwardSysIdQuasi() {
        return controller.start().and(controller.y());
    }

    @Override public Trigger backwardSysIdQuasi() {
        return controller.start().and(controller.x());
    }

    @Override public Trigger forwardSysIdDynamic() {
        return controller.back().and(controller.y());
    }

    @Override public Trigger backwardSysIdDynamic() {
        return controller.back().and(controller.x());
    }
}
