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
    private static final double MAX_SAFE_DIST_TO_PATH = 1.0;

    private final Drivetrain drivetrain;
    private final List<PathPlannerPath> pathList = new ArrayList<>(4);
    private final List<Command> commandPathList = new ArrayList<>(4);

    private Command closestCommand;
    private boolean error = false;

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
        if (error) return; // Exit early if paths failed to load

        List<Translation2d> translationsList = new ArrayList<>();

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

        closestCommand = findClosestCommand(translationsList);

        if (closestCommand != null) {
            CommandScheduler.getInstance().schedule(closestCommand);
        }
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if (closestCommand != null) {
            CommandScheduler.getInstance().cancel(closestCommand);
        }
    }

    @Override
    public boolean isFinished() {
        // Abort immediately if there was a load error or no valid path was found
        if (error || closestCommand == null) {
            return true;
        }
        // The command is finished when the scheduled path-following command is no longer running
        return !CommandScheduler.getInstance().isScheduled(closestCommand);
    }

    private Command findClosestCommand(List<Translation2d> translationList) {
        if (translationList.isEmpty()) {
            DriverStation.reportError("ERROR: List of path starting positions is empty.", false);
            return null;
        }

        Translation2d currentTranslation = drivetrain.getState().Pose.getTranslation();

        double closestDistance = currentTranslation.getDistance(translationList.get(0));
        int closestIndex = 0;

        for (int index = 1; index < translationList.size(); index++) {
            double distance = currentTranslation.getDistance(translationList.get(index));
            if (distance < closestDistance) {
                closestDistance = distance;
                closestIndex = index;
            }
        }

        if (closestDistance < MAX_SAFE_DIST_TO_PATH) {
            return commandPathList.get(closestIndex);
        } else {
            DriverStation.reportWarning(
                "COMMAND NOT SCHEDULED: No paths found within " + MAX_SAFE_DIST_TO_PATH + " meters.", false);
            return null;
        }
    }
}
