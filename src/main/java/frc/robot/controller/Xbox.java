package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Xbox extends Controller {
    private final CommandXboxController controller;

    /** Uses {@link CommandXboxController}. @param port index on Driver Station */
    public Xbox(int port) {
        controller = new CommandXboxController(port);
    }

    @Override
    Translation2d getTranslation() {
        return applyRadialDeadzone(-controller.getLeftY(), -controller.getLeftX(), DEADZONE);
    }

    @Override
    double getRotation() {
        return MathUtil.applyDeadband(-controller.getRightX(), DEADZONE);
    }

    @Override
    Trigger activateIntake() {
        return controller.leftTrigger();
    }

    @Override
    Trigger shootFuel() {
        return controller.b(); // change to right trigger
    }

    @Override
    Trigger rotateToHub() {
        return controller.a();
    }

    @Override
    Trigger lowerIntake() {
        return controller.rightBumper();
    }

    @Override
    Trigger raiseIntake() {
        return controller.leftBumper();
    }

    @Override
    Trigger povUp() {
        return controller.povUp();
    }

    @Override
    Trigger povDown() {
        return controller.povDown();
    }

    @Override
    Trigger povLeft() {
        return controller.povLeft();
    }

    @Override
    Trigger povRight() {
        return controller.povRight();
    }

    @Override
    Trigger forwardSysIdQuasi() {
        return controller.start().and(controller.y());
    }

    @Override
    Trigger backwardSysIdQuasi() {
        return controller.start().and(controller.x());
    }

    @Override
    Trigger forwardSysIdDynamic() {
        return controller.back().and(controller.y());
    }

    @Override
    Trigger backwardSysIdDynamic() {
        return controller.back().and(controller.x());
    }
}
