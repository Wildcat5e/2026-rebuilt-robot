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
    public Translation2d getTranslation() {
        return applyRadialDeadzone(-controller.getLeftY(), -controller.getLeftX(), DEADZONE);
    }

    @Override
    public double getRotation() {
        return MathUtil.applyDeadband(-controller.getRightX(), DEADZONE);
    }

    @Override
    public Trigger activateIntake() {
        return controller.leftTrigger();
    }

    @Override
    public Trigger shootFuel() {
        return controller.b(); // change to right trigger
    }

    @Override
    public Trigger rotateToHub() {
        return controller.a();
    }

    @Override
    public Trigger lowerIntake() {
        return controller.rightBumper();
    }

    @Override
    public Trigger raiseIntake() {

        return controller.leftBumper();
    }

    @Override
    public Trigger forwardSysIdQuasi() {
        return controller.start().and(controller.y());
    }

    @Override
    public Trigger backwardSysIdQuasi() {
        return controller.start().and(controller.x());
    }

    @Override
    public Trigger forwardSysIdDynamic() {
        return controller.back().and(controller.y());
    }

    @Override
    public Trigger backwardSysIdDynamic() {
        return controller.back().and(controller.x());
    }
}
