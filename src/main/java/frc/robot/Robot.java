package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
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

    /** This function is run when the robot is first started up and should be used for any initialization code. */
    public Robot() {
        NamedCommands.registerCommand("Rotate To Hub", RobotCommands.rotateToHub);
        Controller.drivetrain.configureAutoBuilder();
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
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        field.setRobotPose(Controller.drivetrain.getState().Pose);
    }

    @Override
    public void autonomousInit() {
        elasticTabPublisher.set("Autonomous");
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().schedule(autoChooser.getSelected());
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        elasticTabPublisher.set("Teleoperated");
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
}
