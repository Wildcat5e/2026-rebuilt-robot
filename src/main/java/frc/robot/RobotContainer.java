package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ControllerWrapper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;

/** Used for Robot Setup. Lots of static methods and variables */
public interface RobotContainer {
    /** kSpeedAt12Volts desired top speed */
    double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** 3/4 revs per sec max angular velocity in radians per second */
    double MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /** The only instance of Drivetrain. */
    Drivetrain drivetrain = TunerConstants.createDrivetrain();
    /** Setting up bindings for necessary control of the swerve drive platform */
    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    /** The only instance of PhotonVision. */
    PhotonVision photonVision = new PhotonVision(drivetrain);
    /** The only instance of the Xbox Controller. */
    CommandXboxController joystick = new CommandXboxController(0);
    /** The only instance of the Controller. */
    ControllerWrapper controller = new ControllerWrapper.Xbox(0);

    /** Dashboard field widget */
    Field2d field = new Field2d();

    /** Sets up key/button/joystick bindings for driving and controlling the robot. */
    static void bindingsSetup() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            Translation2d translation = controller.getTranslation();
            return drive.withVelocityX(translation.getX() * MAX_LINEAR_SPEED)
                .withVelocityY(translation.getY() * MAX_LINEAR_SPEED)
                .withRotationalRate(controller.getRotation() * MAX_ANGULAR_SPEED);
        }));



        // reset the field-centric heading on left trigger
        joystick.leftTrigger().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.a().whileTrue(Commands.rotateToHub);

        // cancel auto align command
        // joystick.rightBumper().onTrue(new InstantCommand(Commands.autoAlign::cancel));

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

    static void dashboardUpdate() {
        field.setRobotPose(drivetrain.getState().Pose);
    }

    static void generalSetup() {
        SmartDashboard.putData("Field", field);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand()); // replaces: PathfindingCommand.warmupCommand().schedule();
        SignalLogger.enableAutoLogging(false);
    }

    static void runCommands() {
        CommandScheduler.getInstance().run();
    }
}
