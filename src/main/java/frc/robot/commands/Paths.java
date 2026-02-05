package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class Paths extends Command {
    private final Drivetrain drivetrain;
    List<Translation2d> translationsList;
    boolean error = false;
    ArrayList<Command> commandPathList = new ArrayList<>(4);
    Translation2d currentTranslation;
    Command closestCommand;
    int closestIndex;


    /** Creates a new Paths. */
    public Paths(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        try {
            PathPlannerPath Mid_Top_Bump_DS_Path = PathPlannerPath.fromPathFile("Mid Top Bump DS");
            PathPlannerPath DS_Top_Bump_Mid_Path = PathPlannerPath.fromPathFile("DS Top Bump Mid");
            PathPlannerPath Mid_Bottom_Bump_DS_Path = PathPlannerPath.fromPathFile("Mid Bottom Bump DS");
            PathPlannerPath DS_Bottom_Bump_Mid_Path = PathPlannerPath.fromPathFile("DS Bottom Bump Mid");
            // add warmup command 
            commandPathList.add(AutoBuilder.followPath(Mid_Top_Bump_DS_Path));
            commandPathList.add(AutoBuilder.followPath(DS_Top_Bump_Mid_Path));
            commandPathList.add(AutoBuilder.followPath(Mid_Bottom_Bump_DS_Path));
            commandPathList.add(AutoBuilder.followPath(DS_Bottom_Bump_Mid_Path));
            error = false;
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            error = true;
        }
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!error) {
            if (Robot.alliance == Alliance.Blue) { // @formatter:off
                translationsList = List.of(
                    new Translation2d(6.395, 7.32),
                    new Translation2d(2.404, 7.320),
                    new Translation2d(6.395, 0.756),
                    new Translation2d(2.342, 0.731)); 
            } else {
                translationsList = List.of(
                    new Translation2d(10.145, 0.75), 
                    new Translation2d(14.198, 0.75),
                    new Translation2d(10.145, 7.314), 
                    new Translation2d(14.198, 7.339)); // @formatter:on
            }

            closestIndex = findclosestIndex(commandPathList, translationsList);
            // closestIndex being -1 means distance too far, so only run when it is not -1
            if (closestIndex != -1) {
                closestCommand = commandPathList.get(closestIndex);
                CommandScheduler.getInstance().schedule(closestCommand);
            }
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (closestIndex != -1) {
            CommandScheduler.getInstance().cancel(closestCommand);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    int findclosestIndex(List<Command> commandPathList, List<Translation2d> translationList) {
        double closestDistance = Double.POSITIVE_INFINITY;
        int closestIndex = -1;
        currentTranslation = drivetrain.getState().Pose.getTranslation();
        for (int index = 0; index < 4; index++) {
            double distance = currentTranslation.getDistance(translationList.get(index));
            if (distance < closestDistance) {
                closestDistance = distance;
                closestIndex = index;
            }
        }
        // only return correct index when within 1 meter, more than 1 meter distance could be too far
        // return -1 if too far
        if (closestDistance < 1) {
            return closestIndex;
        } else {
            System.out.println("TOO FAR, GREATER THAN 1 METER");
            return -1;
        }
    }

}
