package frc.robot;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.RotateToHub;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.PhotonVision;
import frc.robot.commands.Paths;
import frc.robot.commands.RobotCommands;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    /** The only instance of the Controller. */
    final Controller controller = new Controller.Xbox(0);
    /** The only instance of PhotonVision. */
    final PhotonVision photonVision = new PhotonVision(Controller.drivetrain::addVisionMeasurement);
    /** Dashboard field widget */
    final Field2d field = new Field2d();
    /** A chooser for autonomous commands */
    final SendableChooser<Command> autoChooser;
    /** https://github.com/Gold872/elastic_dashboard/blob/v2026.1.1/elasticlib/Elastic.java */
    public final StringTopic elasticTabTopic = NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
    public final StringPublisher elasticTabPublisher = elasticTabTopic.publish(PubSubOption.keepDuplicates(true));

    static public Alliance alliance;

    /** This function is run when the robot is first started up and should be used for any initialization code. */
    public Robot() {
        NamedCommands.registerCommand("Rotate To Hub", RobotCommands.rotateToHub);
        configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand()); // replaces: PathfindingCommand.warmupCommand().schedule();
        SignalLogger.enableAutoLogging(false);
        controller.bindingsSetup();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Auto Command Chooser", autoChooser);
        SmartDashboard.putData("RotateToHub PID Controller", RotateToHub.PID_CONTROLLER);
        SmartDashboard.putBoolean("Within Shooting Angle", Utilities.withinShootingAngle());
        SmartDashboard.putBoolean("Within Shooting Distance", Utilities.withinShootingDistance());
        NamedCommands.registerCommand("Rotate To Hub", RobotCommands.rotateToHub);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        field.setRobotPose(Controller.drivetrain.getState().Pose);
    }

    @Override
    public void autonomousInit() {
        // elasticTabPublisher.set("Autonomous");
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().schedule(autoChooser.getSelected());
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // ADD TO AUTONOMOUS INIT, MAKE SURE AT COMP AUTO ISNT STARTING MODE WHEN ROBOT IS DISABLED
        DriverStation.getAlliance().ifPresent(fms_alliance -> alliance = fms_alliance);
        // elasticTabPublisher.set("Teleoperated");
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().cancel(autoChooser.getSelected());
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    private Simulation simulation;

    @Override
    public void simulationInit() {
        simulation = new Simulation();
    }

    @Override
    public void simulationPeriodic() {
        simulation.poseUpdate();
    }

    public void configureAutoBuilder() {// @formatter:off
        try {
            var applyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> Controller.drivetrain.getState().Pose,   // Supplier of current robot pose
                Controller.drivetrain::resetPose,         // Consumer for seeding pose against auto
                () -> Controller.drivetrain.getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> Controller.drivetrain.setControl(
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
                Controller.drivetrain // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }// @formatter:on
}
