package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    /** Get the X axis value. @return The axis value. */
    public double getX();

    /** Get the Y axis value. @return The axis value. */
    public double getY();

    /** Get the rotation axis value. @return The axis value. */
    public double getRotation();

    public

    static class Xbox implements ControllerWrapper {
        private final CommandXboxController controller;

        /** Uses {@link CommandXboxController}. @param port index on Driver Station */
        public Xbox(int port) {
            controller = new CommandXboxController(port);
        }

        @Override
        public double getX() {
            return -controller.getLeftY();
        }

        @Override
        public double getY() {
            return -controller.getLeftX();
        }

        @Override
        public double getRotation() {
            return -controller.getRightX();
        }

    }
    static class LogitechFlightStick implements ControllerWrapper {
        private final CommandJoystick controller;

        /** Uses {@link CommandJoystick} for Logitech Extreme 3D Pro. @param port index on Driver Station */
        public LogitechFlightStick(int port) {
            controller = new CommandJoystick(port);
        }

        @Override
        public double getX() {
            return controller.getRawAxis(1);
        }

        @Override
        public double getY() {
            return controller.getRawAxis(0);
        }

        @Override
        public double getRotation() {
            return -controller.getRawAxis(2);
        }
    }
}
