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
import frc.robot.commands.RobotCommands;
import frc.robot.controller.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

import static frc.robot.Utilities.*;

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
    private final SwerveRequest.FieldCentric controllerSwerveReq =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final Controller controller = IS_COMPETITION ? new Xbox(0) : new MultiController();
    private final OperatorConsole operatorConsole = new OperatorConsole();
    private final PhotonVision photonVision = new PhotonVision(drivetrain::addVisionMeasurement);

    private final Field2d fieldWidget = new Field2d();
    private final SendableChooser<Command> autoChooser;
    /** https://github.com/Gold872/elastic_dashboard/blob/v2026.1.1/elasticlib/Elastic.java */
    private final StringTopic elasticTabTopic =
        NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
    private final StringPublisher elasticTabPublisher = elasticTabTopic.publish(PubSubOption.keepDuplicates(true));
    /** Contains all the commands we use and needs to be instantiated after running {@link #configureAutoBuilder()}. */
    public final Flywheel flywheel = new Flywheel(drivetrain);
    public final Hopper hopper = new Hopper();
    public final Intake intake = new Intake();
    private final RobotCommands commands;

    public static boolean isBlueAlliance = true; // Default to Blue

    /** This function is run when the robot is first started up and should be used for any initialization code. */
    public Robot() {
        configureAutoBuilder();

        commands = new RobotCommands(drivetrain, controllerSwerveReq, flywheel, hopper);
        controller.bindingsSetup(drivetrain, controllerSwerveReq, commands, flywheel, hopper, intake);
        operatorConsole.bindMacropad(commands, flywheel, intake, hopper);

        NamedCommands.registerCommand("Drop Intake", intake.bumpExtenderDownNoLock());
        NamedCommands.registerCommand("Run Intake", intake.spinIntakeMotors());
        NamedCommands.registerCommand("Aim at Hub", commands.aimAtHub);
        NamedCommands.registerCommand("Shoot Fuel", commands.shootFuel);
        SignalLogger.enableAutoLogging(false);

        autoChooser = AutoBuilder.buildAutoChooser();
        DashboardManager.setupRobotInit(fieldWidget, autoChooser, drivetrain);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        fieldWidget.setRobotPose(drivetrain.getState().Pose);
        DashboardManager.updateRobotPeriodic(drivetrain, getHubPosition());
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
