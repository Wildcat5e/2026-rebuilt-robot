package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class Paths extends Command {
    /** Maximum allowable distance (meters) from the robot's current pose to a path's starting pose. */
    private static final double MAX_DIST_FROM_PATH = 1.0;
    private final Drivetrain drivetrain;
    List<Translation2d> translationsList;
    List<PathPlannerPath> pathList = new ArrayList<>(4);
    List<Command> commandPathList = new ArrayList<>(4);
    Translation2d currentTranslation;
    Command closestCommand;
    int closestIndex;
    boolean error = false;

    public Paths(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        try {
            PathPlannerPath Mid_Top_Bump_DS_Path = PathPlannerPath.fromPathFile("Mid Top Bump DS");
            PathPlannerPath DS_Top_Bump_Mid_Path = PathPlannerPath.fromPathFile("DS Top Bump Mid");
            PathPlannerPath Mid_Bottom_Bump_DS_Path = PathPlannerPath.fromPathFile("Mid Bottom Bump DS");
            PathPlannerPath DS_Bottom_Bump_Mid_Path = PathPlannerPath.fromPathFile("DS Bottom Bump Mid");

            // Store the paths so we can access their translations in initialize()
            pathList.add(Mid_Top_Bump_DS_Path);
            pathList.add(DS_Top_Bump_Mid_Path);
            pathList.add(Mid_Bottom_Bump_DS_Path);
            pathList.add(DS_Bottom_Bump_Mid_Path);

            commandPathList.add(AutoBuilder.followPath(Mid_Top_Bump_DS_Path));
            commandPathList.add(AutoBuilder.followPath(DS_Top_Bump_Mid_Path));
            commandPathList.add(AutoBuilder.followPath(Mid_Bottom_Bump_DS_Path));
            commandPathList.add(AutoBuilder.followPath(DS_Bottom_Bump_Mid_Path));
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            error = true;
        }
        // Use addRequirements() here to declare subsystem dependencies.
    }

    @Override
    public void initialize() {
        if (!error) {
            translationsList = new ArrayList<>();

            // Loop through the paths to extract coordinates
            for (PathPlannerPath path : pathList) {

                // Get the starting pose (this always returns Blue Alliance coordinate)
                Pose2d bluePose = path.getStartingHolonomicPose().orElse(new Pose2d());
                Translation2d translation = bluePose.getTranslation();

                // If we're on the Red Alliance, make PathPlanner flip the coordinate dynamically
                if (!Robot.isBlueAlliance) {
                    translation = FlippingUtil.flipFieldPosition(translation);
                }

                translationsList.add(translation);
            }

            closestIndex = findclosestIndex(commandPathList, translationsList);
            // A closestIndex of -1 means the distance is too large
            if (closestIndex != -1) {
                closestCommand = commandPathList.get(closestIndex);
                CommandScheduler.getInstance().schedule(closestCommand);
            } else DriverStation
                .reportWarning("Distance to closest path is greater than " + MAX_DIST_FROM_PATH + " meters.", false);
        }
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if (closestIndex != -1) {
            CommandScheduler.getInstance().cancel(closestCommand);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    int findclosestIndex(List<Command> commandPathList, List<Translation2d> translationList) {
        double closestDistance = Double.POSITIVE_INFINITY;
        int closestIndex = -1;
        currentTranslation = drivetrain.getState().Pose.getTranslation();

        for (int index = 0; index < translationList.size(); index++) {
            double distance = currentTranslation.getDistance(translationList.get(index));
            if (distance < closestDistance) {
                closestDistance = distance;
                closestIndex = index;
            }
        }

        // Return correct index if within safe distance of 1 meter, otherwise return -1
        return closestDistance < 1 ? closestIndex : -1;
    }
}
