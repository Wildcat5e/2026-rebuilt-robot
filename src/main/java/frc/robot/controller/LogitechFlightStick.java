package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechFlightStick implements Controller {
    /**
     * Button numbers match numbers printed on the controller.
     */
    protected final CommandJoystick joystick;

    /**
     * Uses {@link CommandJoystick} for Logitech Extreme 3D Pro. @param port index on Driver Station
     */
    public LogitechFlightStick(int port) {
        joystick = new CommandJoystick(port);
    }

    @Override public Translation2d translation() {
        return new Translation2d(-joystick.getRawAxis(1), -joystick.getRawAxis(0));
    }

    @Override public double rotation() {
        return MathUtil.applyDeadband(-joystick.getRawAxis(2), .3);
    }

    @Override public Trigger runIntake() {return joystick.button(3);}

    @Override public Trigger shootFuel() {return joystick.trigger();}

    @Override public Trigger lowerIntake() {
        return joystick.button(5);
    }

    @Override public Trigger raiseIntake() {
        return joystick.button(6);
    }

    @Override public Trigger aimHandler() {
        return joystick.button(2); // Thumb button, while holding easy to trigger to shoot.
    }

    @Override public Trigger manualFlywheel() {
        return joystick.button(4);
    }

    @Override public Trigger seedFieldCentric() {
        return joystick.button(12);
    }

    @Override public Trigger reverse() {
        return joystick.button(11);
    }

    @Override public Trigger povUp() {
        return joystick.povUp();
    }

    @Override public Trigger povDown() {
        return joystick.povDown();
    }

    @Override public Trigger povLeft() {
        return joystick.povLeft();
    }

    @Override public Trigger povRight() {
        return joystick.povRight();
    }

    @Override public Trigger forwardSysIdQuasi() {return new Trigger(() -> false);}

    @Override public Trigger backwardSysIdQuasi() {return new Trigger(() -> false);}

    @Override public Trigger forwardSysIdDynamic() {return new Trigger(() -> false);}

    @Override public Trigger backwardSysIdDynamic() {return new Trigger(() -> false);}
}
