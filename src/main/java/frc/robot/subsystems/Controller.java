package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.commands.RobotCommands;
import frc.robot.generated.TunerConstants;

/**
 * Interface for standardized Controller use.
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
    static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI;
    static final double MAX_ANGULAR_ACCEL = Constants.MAX_ANGULAR_ACCEL;
    /** The only instance of Drivetrain. */
    public static final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    /** Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric swerveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

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

    /** Sets up key/button/joystick bindings for driving and controlling the robot. */
    public void bindingsSetup() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            Translation2d translation = getTranslation();
            if (allowControllerTranslation) {
                swerveRequest.withVelocityX(translation.getX() * Constants.MAX_LINEAR_SPEED)
                    .withVelocityY(translation.getY() * Constants.MAX_LINEAR_SPEED);
            }
            if (allowControllerRotation) {
                swerveRequest.withRotationalRate(getRotation() * MAX_ANGULAR_SPEED);
            }
            return swerveRequest;
        }));
        // reset the field-centric heading on left trigger
        Controller.joystick.leftTrigger().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        Controller.joystick.a().whileTrue(RobotCommands.rotateToHub);

        /*
         * Tests for motor identification:
         * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
         * https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration
         */
        // Quasistatic test for motor identification
        Controller.joystick.start().and(Controller.joystick.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Controller.joystick.start().and(Controller.joystick.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // Dynamic test for motor identification
        Controller.joystick.back().and(Controller.joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Controller.joystick.back().and(Controller.joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    }

    /** Call to update values before calling getX() or getY(). */
    public abstract Translation2d getTranslation();

    /** Get the rotation axis value. @return The axis value. */
    public abstract double getRotation();

    public static class Xbox extends Controller {
        private final CommandXboxController controller;

        /** Uses {@link CommandXboxController}. @param port index on Driver Station */
        public Xbox(int port) {
            super();
            controller = new CommandXboxController(port);
        }

        @Override
        public Translation2d getTranslation() {
            return applyRadialDeadzone(controller.getLeftY(), controller.getLeftX(), DEADZONE);
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
            super();
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

}
