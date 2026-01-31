package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.commands.Paths;
import frc.robot.commands.RobotCommands;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    static public Alliance alliance;

    /** This function is run when the robot is first started up and should be used for any initialization code. */
    public Robot() {
        RobotContainer.generalSetup();
        RobotContainer.bindingsSetup();
        NamedCommands.registerCommand("Rotate To Hub", RobotCommands.rotateToHub);
    }

    @Override
    public void robotPeriodic() {
        RobotContainer.runCommands();
        RobotContainer.dashboardUpdate();
    }

    @Override
    public void autonomousInit() {
        // RobotContainer.elasticTabPublisher.set(RobotContainer.ELASTIC_AUTONOMOUS);
        RobotContainer.runAuto();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // ADD TO AUTONOMOUS INIT, MAKE SURE AT COMP AUTO ISNT STARTING MODE WHEN ROBOT IS DISABLED
        DriverStation.getAlliance().ifPresent(fms_alliance -> alliance = fms_alliance);
        // RobotContainer.elasticTabPublisher.set(RobotContainer.ELASTIC_TELEOP);
        RobotContainer.cancelAuto();
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
