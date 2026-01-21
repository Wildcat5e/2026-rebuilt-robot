package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Interface for standardized Controller use.
 * 
 * @apiNote Each axis uses the
 *          <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">WPILib
 *          coordinate system<a>.
 */
public interface ControllerWrapper {
    /** Deadzone to apply to joysticks as a proportion out of 1. */
    double DEADZONE = .1;


    /** Get the X axis value. @return The axis value. */
    double getX();

    /** Get the Y axis value. @return The axis value. */
    double getY();

    /** Get the rotation axis value. @return The axis value. */
    double getRotation();

    /** Get magnitude of the vector of X and  Y. */
    double getVectorMagnitude();

    static class Xbox implements ControllerWrapper {
        private final CommandXboxController controller;

        /** Uses {@link CommandXboxController}. @param port index on Driver Station */
        public Xbox(int port) {
            controller = new CommandXboxController(port);
        }

        @Override
        public double getX() {
            return applyDeadzone(-controller.getLeftY(), getVectorMagnitude(), DEADZONE);
        }

        @Override
        public double getY() {
            return applyDeadzone(-controller.getLeftX(), getVectorMagnitude(), DEADZONE);
        }

        @Override
        public double getRotation() {
            return applyDeadzone(-controller.getRightX(), controller.getRightX(), DEADZONE);
        }
        
        @Override
        public double getVectorMagnitude() {
            return Math.hypot(controller.getLeftY(), controller.getLeftX());
        }
    }
    static class LogitechFlightStick implements ControllerWrapper {
        private final CommandJoystick controller;
        /** Deadzone specific to flight stick. */
        private static final double DEADZONE = ControllerWrapper.DEADZONE; // for now use main deadzone

        /** Uses {@link CommandJoystick} for Logitech Extreme 3D Pro. @param port index on Driver Station */
        public LogitechFlightStick(int port) {
            controller = new CommandJoystick(port);
        }

        @Override
        public double getX() {
            return applyDeadzone(controller.getRawAxis(1), getVectorMagnitude(), DEADZONE);
        }

        @Override
        public double getY() {
            return applyDeadzone(controller.getRawAxis(0), getVectorMagnitude(), DEADZONE);
        }

        @Override
        public double getRotation() {
            return applyDeadzone(-controller.getRawAxis(2), controller.getRawAxis(2), DEADZONE);
        }
        
        @Override
        public double getVectorMagnitude() {
            return Math.hypot(controller.getRawAxis(1), controller.getRawAxis(0));
        }
    }

    /**
     * APPLY FIRST! Applies a deadzone as a proportion of the input. Values shifted up out of deadzone and compressed
     * outside deadzone. The max value of 1 remains at the max. This is a scaled radial deadzone.
     * 
     * @param axisValue raw value from controller
     * @param vectorMagnitude positive value of the magnitude of the vector formed by two inputs
     * @param deadZone proportion to eliminate
     * @return axis value with zero above deadzone
     */
    static double applyDeadzone(double axisValue, double vectorMagnitude, double deadZone) {
        if (vectorMagnitude) < deadZone) {
            return 0;
        } else if (axisValue > 0) {
            return 1 / (1 - deadZone) * axisValue - deadZone;
        } else
            return 1 / (1 - deadZone) * axisValue + deadZone;
    }
}
