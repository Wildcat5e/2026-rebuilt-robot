package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public interface ControllerWrapper {
    public double getX();

    public double getY();

    public double getRotation();

    static class Xbox implements ControllerWrapper {
        private final XboxController controller;

        public Xbox(int port) {
            controller = new XboxController(port);
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
        private final Joystick controller;

        public LogitechFlightStick(int port) {
            controller = new Joystick(port);
        }

        @Override
        public double getX() {
            return controller.getRawAxis(0);
        }

        @Override
        public double getY() {
            return controller.getRawAxis(1);
        }

        @Override
        public double getRotation() {
            return controller.getRawAxis(2);
        }
    }
}
