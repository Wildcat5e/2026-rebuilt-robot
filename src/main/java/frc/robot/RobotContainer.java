package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RotateToHub;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.PhotonVision;

/** Used for Robot Setup. Lots of static methods and variables */
public interface RobotContainer {
    /** The only instance of PhotonVision. */
    PhotonVision photonVision = new PhotonVision(Controller.drivetrain::addVisionMeasurement);
    /** The only instance of the Controller. */
    Controller controller = new Controller.Xbox(0);

    /** Dashboard field widget */
    Field2d field = new Field2d();
    /** A chooser for autonomous commands */
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    /** https://github.com/Gold872/elastic_dashboard/blob/v2026.1.1/elasticlib/Elastic.java */
    StringTopic elasticTabTopic = NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
    StringPublisher elasticTabPublisher = elasticTabTopic.publish(PubSubOption.keepDuplicates(true));
    String ELASTIC_TELEOP = "Teleoperated";
    String ELASTIC_AUTONOMOUS = "Autonomous";

    static void dashboardUpdate() {
        field.setRobotPose(Controller.drivetrain.getState().Pose);
    }

    static void generalSetup() {
        controller.bindingsSetup();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Auto Command Chooser", autoChooser);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand()); // replaces: PathfindingCommand.warmupCommand().schedule();
        SignalLogger.enableAutoLogging(false);
        SmartDashboard.putData("RotateToHub PID Controller", RotateToHub.PID_CONTROLLER);
        SmartDashboard.putBoolean("Within Shooting Angle", Utilities.withinShootingAngle());
        SmartDashboard.putBoolean("Within Shooting Distance", Utilities.withinShootingDistance());
    }

    static void runCommands() {
        CommandScheduler.getInstance().run();
    }

    static void runAuto() {
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().schedule(autoChooser.getSelected());
        }
    }

    static void cancelAuto() {
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().cancel(autoChooser.getSelected());
        }
    }
}
