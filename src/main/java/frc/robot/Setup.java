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
class Setup {
    /** kSpeedAt12Volts desired top speed */
    static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** 3/4 of a rotation per second max angular velocity */
    static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /** The only instance of Drivetrain. */
    static final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    /** The only instance of the Xbox Controller. */
    static final CommandXboxController joystick = new CommandXboxController(0);
    /** Setting up bindings for necessary control of the swerve drive platform */
    static final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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
            drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
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
