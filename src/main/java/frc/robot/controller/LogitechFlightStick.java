package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechFlightStick extends Controller {
    /** Button numbers match numbers printed on the controller. */
    protected final CommandJoystick controller;
    /** Deadzone specific to flight stick. */
    // private static final double DEADZONE = ControllerWrapper.DEADZONE; // for now use main deadzone

    /** Uses {@link CommandJoystick} for Logitech Extreme 3D Pro. @param port index on Driver Station */
    public LogitechFlightStick(int port) {
        controller = new CommandJoystick(port);
    }

    @Override
    Translation2d getTranslation() {
        return applyRadialDeadzone(-controller.getRawAxis(1), -controller.getRawAxis(0), DEADZONE);
    }

    @Override
    double getRotation() {
        return MathUtil.applyDeadband(-controller.getRawAxis(2), .3);
    }

    @Override
    Trigger runIntake() {
        return controller.button(3); // also consider button 10
    }

    @Override
    Trigger shootFuel() {
        return controller.trigger(); // == controller.button(1)
    }

    @Override
    Trigger lowerIntake() {
        return controller.button(5);
    }

    @Override
    Trigger raiseIntake() {
        return controller.button(6);
    }

    @Override
    Trigger aimHandler() {
        return controller.button(2); // Thumb button, while holding easy to trigger to shoot.
    }

    @Override
    Trigger manualFlywheel() {
        return controller.button(4);
    }

    @Override
    Trigger seedFieldCentric() {
        return controller.button(12);
    }

    @Override
    Trigger reverse() {
        return controller.button(11);
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
        /* change this */ return new Trigger(() -> false);
        // return Controller.joystick.start().and(Controller.joystick.y());
    }

    @Override
    Trigger backwardSysIdQuasi() {
        /* change this */ return new Trigger(() -> false);
        // return Controller.joystick.start().and(Controller.joystick.x());
    }

    @Override
    Trigger forwardSysIdDynamic() {
        /* change this */ return new Trigger(() -> false);
        // return Controller.joystick.back().and(Controller.joystick.y());
    }

    @Override
    Trigger backwardSysIdDynamic() {
        /* change this */ return new Trigger(() -> false);
        // return Controller.joystick.back().and(Controller.joystick.x());
    }
}
