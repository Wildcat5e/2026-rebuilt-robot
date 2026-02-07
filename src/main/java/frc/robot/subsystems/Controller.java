package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/**
 * Controller management.
 * 
 * @apiNote Each axis uses the
 *          <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">WPILib
 *          coordinate system<a>.
 */
public abstract class Controller {
    /** Deadzone to apply to joysticks as a proportion out of 1. */
    static final double DEADZONE = .15;
    /** Exponent to raise inputs to the power of to create a curved response. */
    static final double SCALE_EXPONENT = 1;
    public static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI;
    public static final double MAX_ANGULAR_ACCEL = Constants.MAX_ANGULAR_ACCEL;
    /** Change whether or not controller can control translation. */
    public static boolean allowControllerTranslation = true;
    /** Change whether or not controller can control rotation. */
    public static boolean allowControllerRotation = true;

    /**
     * APPLY FIRST! Applies a deadzone as a proportion of the input. Values shifted up out of deadzone and compressed
     * outside deadzone. The max value of 1 remains at the max. This is a scaled radial deadzone. Also, curves input.
     * 
     * @param xAxis raw value from controller
     * @param yAxis raw value from controller
     * @param deadzone proportion to eliminate
     * @return axis values in Translation2d
     */
    static Translation2d applyRadialDeadzone(double xAxis, double yAxis, double deadzone) {
        double magnitude = Math.hypot(xAxis, yAxis);
        if (magnitude < deadzone) {
            return new Translation2d(0, 0);
        }
        double scaledMagnitude = Math.pow(MathUtil.applyDeadband(magnitude, deadzone), SCALE_EXPONENT);
        return new Translation2d(xAxis, yAxis).div(magnitude).times(scaledMagnitude);
    }

    /** The only instance of the Xbox Controller. */
    public static final CommandXboxController joystick = new CommandXboxController(0);

    /** Get Translation2d of controller axes. */
    public abstract Translation2d getTranslation();

    /** Get the rotation axis value. @return The axis value. */
    public abstract double getRotation();

    public static class Xbox extends Controller {
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
    }
    public static class LogitechFlightStick extends Controller {
        private final CommandJoystick controller;
        /** Deadzone specific to flight stick. */
        // private static final double DEADZONE = ControllerWrapper.DEADZONE; // for now use main deadzone

        /** Uses {@link CommandJoystick} for Logitech Extreme 3D Pro. @param port index on Driver Station */
        public LogitechFlightStick(int port) {
            controller = new CommandJoystick(port);
        }

        @Override
        public Translation2d getTranslation() {
            return applyRadialDeadzone(controller.getRawAxis(1), controller.getRawAxis(0), DEADZONE);
        }

        @Override
        public double getRotation() {
            return MathUtil.applyDeadband(-controller.getRawAxis(2), .3);
        }
    }
    public static class Keyboard extends Controller {

        private final CommandJoystick controller;
        /** Deadzone specific to flight stick. */
        // private static final double DEADZONE = ControllerWrapper.DEADZONE; // for now use main deadzone

        /** Uses {@link CommandJoystick} for Logitech Extreme 3D Pro. @param port index on Driver Station */
        public Keyboard(int port) {
            controller = new CommandJoystick(port);
        }

        @Override
        public Translation2d getTranslation() {
            return applyRadialDeadzone(controller.getRawAxis(1), controller.getRawAxis(0), DEADZONE);
        }

        @Override
        public double getRotation() {
            return MathUtil.applyDeadband(-controller.getRawAxis(2), .3);
        }
    }
    public static class MultiController extends Controller {
        final SendableChooser<Controller> controllerChooser = new SendableChooser<Controller>();

        public MultiController() {
            controllerChooser.setDefaultOption("Xbox Controller", new Controller.Xbox(0));
            controllerChooser.addOption("Logitech Flight Stick", new Controller.LogitechFlightStick(1));
            controllerChooser.addOption("Keyboard", new Controller.Keyboard(2));
            SmartDashboard.putData("Controller Chooser", controllerChooser);
        }

        @Override
        public double getRotation() {
            return controllerChooser.getSelected().getRotation();
        }

        @Override
        public Translation2d getTranslation() {
            return controllerChooser.getSelected().getTranslation();
        }
    }
}
