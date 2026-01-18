package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

/** Used for Robot Setup. Lots of static methods and variables */
class RobotContainer {
    /** kSpeedAt12Volts desired top speed */
    static final double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** 3/4 revs per sec max angular velocity in radians per second */
    static final double MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /** The only instance of Drivetrain. */
    static final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    /** The only instance of the Xbox Controller. */
    static final CommandXboxController joystick = new CommandXboxController(0);
    /** Setting up bindings for necessary control of the swerve drive platform */
    static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MAX_LINEAR_SPEED * 0.1).withRotationalDeadband(MAX_ANGULAR_SPEED * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    /** Dashboard field widget */
    static final Field2d field = new Field2d();

    /**
     * Sets up key/button/joystick bindings for driving and controlling the robot.
     * 
     * @apiNote Called once by {@link Robot#Robot Robot}.
     */
    static void bindingsSetup() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MAX_LINEAR_SPEED) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MAX_LINEAR_SPEED) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MAX_ANGULAR_SPEED) // Drive counterclockwise with negative X (left)
            ));



        // reset the field-centric heading on left trigger
        joystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // Isn't drivetrain::seedFieldCentric equivalent?
        // auto align with hub on left bumper press
        // joystick.leftBumper().onTrue(autoAlignCommands.leftAutoAlign());

        /*
         * Tests for motor identification:
         * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
         * https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration
         */
        // Quasistatic test for motor identification
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // Dynamic test for motor identification
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    }

    /**
     * Set up dashboard widgets
     * 
     * @apiNote Called once by {@link Robot#Robot Robot}.
     */
    static void dashboardSetup() {
        SmartDashboard.putData("Field", field);
    }

    /**
     * Update dashboard widgets
     * 
     * @apiNote Called periodically by {@link Robot#robotPeriodic()}.
     */
    static void dashboardUpdate() {
        field.setRobotPose(drivetrain.getState().Pose);
    }



}
