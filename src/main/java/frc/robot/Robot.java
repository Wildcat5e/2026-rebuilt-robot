package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    /** The only instance of Drivetrain. */
    private final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    /** Use this to create requests for driving the robot and use {@link #drivetrain} to apply them. */
    private final SwerveRequest.FieldCentric swerveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final Controller controller = new Controller.MultiController();
    private final PhotonVision photonVision = new PhotonVision(drivetrain::addVisionMeasurement);

    private final Field2d fieldWidget = new Field2d();
    private final SendableChooser<Command> autoChooser;
    /** https://github.com/Gold872/elastic_dashboard/blob/v2026.1.1/elasticlib/Elastic.java */
    private final StringTopic elasticTabTopic =
        NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
    private final StringPublisher elasticTabPublisher = elasticTabTopic.publish(PubSubOption.keepDuplicates(true));
    /** Contains all the commands we use and needs to be instantiated after running {@link #configureAutoBuilder()}. */
    private final Commands commands;

    public static boolean isBlueAlliance = true; // Default to Blue

    /** This function is run when the robot is first started up and should be used for any initialization code. */
    public Robot() {
        configureAutoBuilder();
        commands = new Commands(drivetrain);

        bindingsSetup();
        NamedCommands.registerCommand("Rotate To Hub", commands.rotateToHub);
        SignalLogger.enableAutoLogging(false);

        autoChooser = AutoBuilder.buildAutoChooser();
        DashboardManager.setupRobotInit(fieldWidget, autoChooser, drivetrain);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        fieldWidget.setRobotPose(drivetrain.getState().Pose);
        DashboardManager.updateRobotPeriodic(drivetrain);
    }

    @Override
    public void autonomousInit() {
        DriverStation.getAlliance().ifPresent(fms_alliance -> isBlueAlliance = fms_alliance == Alliance.Blue);
        // REMEMBER TO UNCOMMENT THE LINE BELOW BEFORE COMPETITION!
        // elasticTabPublisher.set("Autonomous");
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().schedule(autoChooser.getSelected());
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        DriverStation.getAlliance().ifPresent(fms_alliance -> isBlueAlliance = fms_alliance == Alliance.Blue);
        // REMEMBER TO UNCOMMENT THE LINE BELOW BEFORE COMPETITION!
        // elasticTabPublisher.set("Teleoperated");
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().cancel(autoChooser.getSelected());
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {
        DriverStation.getAlliance().ifPresent(fms_alliance -> isBlueAlliance = fms_alliance == Alliance.Blue);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    private Simulation simulation;

    @Override
    public void simulationInit() {
        simulation = new Simulation(drivetrain);
    }

    @Override
    public void simulationPeriodic() {
        simulation.poseUpdate();
    }

    /** Sets up key/button/joystick bindings for driving and controlling the robot. */
    public void bindingsSetup() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            Translation2d translation = controller.getTranslation();
            if (Controller.allowControllerTranslation) {
                swerveRequest.withVelocityX(translation.getX() * Constants.MAX_LINEAR_SPEED)
                    .withVelocityY(translation.getY() * Constants.MAX_LINEAR_SPEED);
            }
            if (Controller.allowControllerRotation) {
                swerveRequest.withRotationalRate(controller.getRotation() * Controller.MAX_ANGULAR_SPEED);
            }
            return swerveRequest;
        }));
        // reset the field-centric heading on left trigger
        Controller.joystick.leftTrigger().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Controller.joystick.a().whileTrue(new RotateToHub(drivetrain, true)); // PID + Shooting Calculator testing
        // Controller.joystick.rightBumper().whileTrue(new RotateToHub(drivetrain, false)); // Pure Feedforward + PID testing

        // Controller.joystick.b().whileTrue(flywheel.testTunableFlywheel());

        // Controller.joystick.b().whileTrue(hopper.testTunableKicker());

        Controller.joystick.povUp().whileTrue(commands.flywheel.sysIdDynamicForward());
        Controller.joystick.povRight().whileTrue(commands.flywheel.sysIdDynamicReverse());
        Controller.joystick.povDown().whileTrue(commands.flywheel.sysIdQuasistaticForward());
        Controller.joystick.povLeft().whileTrue(commands.flywheel.sysIdQuasistaticReverse());


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

    /** This configures {@link AutoBuilder} and must be run before creating commands that use it. */
    public void configureAutoBuilder() {// @formatter:off
        try {
            var applyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,   // Supplier of current robot pose
                drivetrain::resetPose,         // Consumer for seeding pose against auto
                () -> drivetrain.getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> drivetrain.setControl(
                    applyRobotSpeedsRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }// @formatter:on
}
