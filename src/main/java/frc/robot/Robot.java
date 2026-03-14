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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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
    public static final boolean IS_COMPETITION = false;
    /** The only instance of Drivetrain. */
    private final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    /** Use this to create requests for driving the robot and use {@link #drivetrain} to apply them. */
    public static final SwerveRequest.FieldCentric swerveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final Controller controller = IS_COMPETITION ? new Controller.Xbox(0) : new Controller.MultiController();

    private final CommandGenericHID macropad = new CommandGenericHID(4);

    private final PhotonVision photonVision = new PhotonVision(drivetrain::addVisionMeasurement);

    private final Field2d fieldWidget = new Field2d();
    private final SendableChooser<Command> autoChooser;
    /** https://github.com/Gold872/elastic_dashboard/blob/v2026.1.1/elasticlib/Elastic.java */
    private final StringTopic elasticTabTopic =
        NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
    private final StringPublisher elasticTabPublisher = elasticTabTopic.publish(PubSubOption.keepDuplicates(true));
    /** Contains all the commands we use and needs to be instantiated after running {@link #configureAutoBuilder()}. */
    private final RobotCommands commands;
    public final Flywheel flywheel = new Flywheel(drivetrain);
    public final Hopper hopper = new Hopper();
    public final Intake intake = new Intake();

    public static boolean isBlueAlliance = true; // Default to Blue

    /** This function is run when the robot is first started up and should be used for any initialization code. */
    public Robot() {
        configureAutoBuilder();
        commands = new RobotCommands(drivetrain, flywheel, intake, hopper);

        bindingsSetup();
        NamedCommands.registerCommand("Drop Intake", intake.bumpExtenderDown());
        NamedCommands.registerCommand("Run Intake", intake.spinIntakeMotors());
        NamedCommands.registerCommand("Shoot Fuel", commands.shootFuel);
        NamedCommands.registerCommand("Rotate To Hub", commands.rotateToHub);
        NamedCommands.registerCommand("Rotate To Hub Shooting Calc", commands.rotateToHubShootingCalc);
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
        if (IS_COMPETITION) elasticTabPublisher.set("Autonomous");
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().schedule(autoChooser.getSelected());
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        DriverStation.getAlliance().ifPresent(fms_alliance -> isBlueAlliance = fms_alliance == Alliance.Blue);
        if (IS_COMPETITION) elasticTabPublisher.set("Teleoperated");
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

        // --- MAIN CONTROLLER BINDINGS ---
        // Controller.joystick.povUp().whileTrue(intake.testExtender());
        // Controller.joystick.povRight().whileTrue(intake.testPusher());
        // Controller.joystick.povDown().whileTrue(intake.testScooper());
        // Controller.joystick.povLeft().whileTrue(hopper.testConveyor());

        // Controller.joystick.b().whileTrue(flywheel.testDynamicStartFlywheel());
        // Controller.joystick.b().whileTrue(hopper.testTunableKicker());

        /** FINAL CONTROL BINDINGS MADE FOR ACTUAL COMPETITION */
        Controller.joystick.rightTrigger().whileTrue(commands.shootFuel);
        Controller.joystick.leftTrigger().whileTrue(intake.spinIntakeMotors());
        Controller.joystick.rightBumper().whileTrue(intake.testExtender());
        Controller.joystick.a().whileTrue(commands.rotateToHubShootingCalc);
        Controller.joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // --- MACROPAD BINDINGS ---
        // LAYER 0 (No Modifiers)
        macropad.button(1).whileTrue(intake.bumpExtenderUp());
        macropad.button(2).whileTrue(intake.bumpExtenderDown());
        macropad.button(3).onTrue(
            Commands.runOnce(() -> DashboardManager.incrementFlywheelSpeedMultiplier(0.01)).ignoringDisable(true));
        macropad.button(4).onTrue(
            Commands.runOnce(() -> DashboardManager.incrementFlywheelSpeedMultiplier(-0.01)).ignoringDisable(true));

        // LAYER 1 (Shift Held)
        macropad.button(5).whileTrue(intake.reverseScooper());
        macropad.button(6).whileTrue(intake.reversePusher());
        macropad.button(7).whileTrue(hopper.reverseConveyor());
        macropad.button(8).whileTrue(hopper.reverseKicker());
        macropad.button(9).whileTrue(flywheel.reverseFlywheel());
        // Emergency Stop
        // macropad.button(10).onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()).ignoringDisable(true));

        // LAYER 2 (Control Held)
        macropad.button(11).whileTrue(hopper.testTunableKicker());
        macropad.button(12).whileTrue(flywheel.backupFlywheelL1());
        macropad.button(13).whileTrue(flywheel.backupFlywheelL2());
        macropad.button(14).whileTrue(flywheel.backupFlywheelL3());
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
