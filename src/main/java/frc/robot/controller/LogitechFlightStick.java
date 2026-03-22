package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechFlightStick extends Controller {
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
    Trigger activateIntake() {
        /* change this */ return controller.button(0);
    }

    @Override
    Trigger shootFuel() {
        return controller.trigger();
    }

    @Override
    Trigger lowerIntake() {
        /* change this */ return controller.button(2);
    }

    @Override
    Trigger raiseIntake() {
        /* change this */ return controller.button(0);
    }

    @Override
    Trigger aimHandler() {
        /* change this */ return controller.button(1);
    }

    @Override
    Trigger manualFlywheel() {
        /* change this */ return controller.button(0);
    }

    @Override
    Trigger seedFieldCentric() {
        /* change this */ return controller.button(0);
    }

    @Override
    Trigger reverse() {
        /* change this */ return controller.button(0);
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
        /* change this */ return controller.button(0);
        // return Controller.joystick.start().and(Controller.joystick.y());
    }

    @Override
    Trigger backwardSysIdQuasi() {
        /* change this */ return controller.button(0);
        // return Controller.joystick.start().and(Controller.joystick.x());
    }

    @Override
    Trigger forwardSysIdDynamic() {
        /* change this */ return controller.button(0);
        // return Controller.joystick.back().and(Controller.joystick.y());
    }

    @Override
    Trigger backwardSysIdDynamic() {
        /* change this */ return controller.button(0);
        // return Controller.joystick.back().and(Controller.joystick.x());
    }
}
