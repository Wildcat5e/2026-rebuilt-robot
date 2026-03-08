package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechFlightStick extends Controller {
    final CommandJoystick controller;
    /** Deadzone specific to flight stick. */
    // private static final double DEADZONE = ControllerWrapper.DEADZONE; // for now use main deadzone

    /** Uses {@link CommandJoystick} for Logitech Extreme 3D Pro. @param port index on Driver Station */
    public LogitechFlightStick(int port) {
        controller = new CommandJoystick(port);
    }

    @Override
    public Translation2d getTranslation() {
        return applyRadialDeadzone(-controller.getRawAxis(1), -controller.getRawAxis(0), DEADZONE);
    }

    @Override
    public double getRotation() {
        return MathUtil.applyDeadband(-controller.getRawAxis(2), .3);
    }

    @Override
    public Trigger activateIntake() {
        /* change this */ return controller.button(0);
    }

    @Override
    public Trigger shootFuel() {
        return controller.trigger();
    }

    @Override
    public Trigger rotateToHub() {
        /* change this */ return controller.button(1);
    }

    @Override
    public Trigger lowerIntake() {
        /* change this */ return controller.button(2);
    }

    @Override
    public Trigger raiseIntake() {
        /* change this */ return controller.button(0);
    }

    @Override
    public Trigger forwardSysIdQuasi() {
        /* change this */ return controller.button(0);
        // return Controller.joystick.start().and(Controller.joystick.y());
    }

    @Override
    public Trigger backwardSysIdQuasi() {
        /* change this */ return controller.button(0);
        // return Controller.joystick.start().and(Controller.joystick.x());
    }

    @Override
    public Trigger forwardSysIdDynamic() {
        /* change this */ return controller.button(0);
        // return Controller.joystick.back().and(Controller.joystick.y());
    }

    @Override
    public Trigger backwardSysIdDynamic() {
        /* change this */ return controller.button(0);
        // return Controller.joystick.back().and(Controller.joystick.x());
    }
}
