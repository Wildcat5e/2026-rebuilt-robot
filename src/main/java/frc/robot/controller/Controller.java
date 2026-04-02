package frc.robot.controller;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

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
    private static final double SCALE_EXPONENT = 1;
    /** Limit max controller angular speed to prevent flicking robot around too fast and spilling balls. */
    private static final double MAX_ANGULAR_SPEED = Constants.MAX_ANGULAR_SPEED - Math.PI; // Check if this is even needed or reasonable.

    /** Change whether or not controller can control translation. */
    public static boolean allowControllerTranslation = true;
    /** Change whether or not controller can control rotation. */
    public static boolean allowControllerRotation = true;

    /** Get vector of controller axes. @return Translation2d of X and Y axes */
    abstract Translation2d getTranslation();

    /** Get the rotation axis value. @return The axis value. */
    abstract double getRotation();

    abstract Trigger runIntake();

    abstract Trigger shootFuel();

    abstract Trigger lowerIntake();

    abstract Trigger raiseIntake();

    abstract Trigger aimHandler();

    abstract Trigger manualFlywheel();

    abstract Trigger seedFieldCentric();

    abstract Trigger reverse();

    abstract Trigger trenchPath();

    abstract Trigger povUp();

    abstract Trigger povDown();

    abstract Trigger povLeft();

    abstract Trigger povRight();

    /** Xbox buttons Start and Y to forwardSysIdQuasi. */
    abstract Trigger forwardSysIdQuasi();

    /** Xbox buttons Start and X to backwardSysIdQuasi. */
    abstract Trigger backwardSysIdQuasi();

    /** Xbox buttons Back and Y to forwardSysIdDynamic. */
    abstract Trigger forwardSysIdDynamic();

    /** Xbox buttons Back and X to backwardSysIdDynamic. */
    abstract Trigger backwardSysIdDynamic();


    /** Sets up key/button/joystick bindings for driving and controlling the robot. */
    public void bindingsSetup(Drivetrain drivetrain, SwerveRequest.FieldCentric swerveRequest, RobotCommands commands,
        Flywheel flywheel, Hopper hopper, Intake intake) {
        /** Competition Bindings */
        shootFuel().whileTrue(commands.feedShooter);
        shootFuel().whileTrue(intake.testPusher());
        runIntake().whileTrue(intake.spinIntakeMotors());
        lowerIntake().whileTrue(intake.bumpExtenderDownNoLock());
        raiseIntake().whileTrue(intake.bumpExtenderUpNoLock());
        aimHandler().whileTrue(commands.aimHandler);
        manualFlywheel().whileTrue(flywheel.tunableFlywheelSpeedCommand());
        seedFieldCentric().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        reverse().whileTrue(intake.reverseScooper());
        trenchPath().whileTrue(intake.zeroExtenderPosition());

        // sysid tests
        // povUp().whileTrue(flywheel.sysIdDynamicForward());
        // povRight().whileTrue(flywheel.sysIdDynamicReverse());
        // povDown().whileTrue(flywheel.sysIdQuasistaticForward());
        // povLeft().whileTrue(flywheel.sysIdQuasistaticReverse());

        /*
         * Tests for motor identification:
         * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
         * https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration
         */
        // // Quasistatic test for motor identification
        // forwardSysIdQuasi().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // backwardSysIdQuasi().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // // Dynamic test for motor identification
        // forwardSysIdDynamic().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // backwardSysIdDynamic().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            var translation = getTranslation();
            if (allowControllerTranslation) {
                swerveRequest.withVelocityX(translation.getX() * Constants.MAX_LINEAR_SPEED)
                    .withVelocityY(translation.getY() * Constants.MAX_LINEAR_SPEED);
            }
            if (allowControllerRotation) {
                swerveRequest.withRotationalRate(getRotation() * MAX_ANGULAR_SPEED);
            }
            return swerveRequest;
        }));
    }

    /**
     * APPLY FIRST! Applies a deadzone as a proportion of the input. Values shifted up out of deadzone and compressed
     * outside deadzone. The max value of 1 remains at the max. This is a scaled radial deadzone. Also, curves input
     * based on {@link #SCALE_EXPONENT}.
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
}
