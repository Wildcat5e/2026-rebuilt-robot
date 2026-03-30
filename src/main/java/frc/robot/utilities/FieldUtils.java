package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/**
 * import static frc.robot.utilities.FieldUtils.*;
 */
public interface FieldUtils {

    /** @return The position of the current Alliance's Hub as a Translation2d. */
    static Translation2d getHubPosition() {
        return Robot.isBlueAlliance ? Constants.BLUE_HUB_POSITION : Constants.RED_HUB_POSITION;
    }

    /** @return The position of the current Alliance's Upper Home as a Translation2d. */
    static Translation2d getUpperHome() {
        return Robot.isBlueAlliance ? Constants.UPPER_HOMES.get(0) : Constants.UPPER_HOMES.get(1);
    }

    /** @return The position of the current Alliance's Lower Home as a Translation2d. */
    static Translation2d getLowerHome() {
        return Robot.isBlueAlliance ? Constants.LOWER_HOMES.get(0) : Constants.LOWER_HOMES.get(1);
    }

    /** @return The position of the current Alliance's closest Home as a Translation2d. */
    static Translation2d getHomeTarget(Drivetrain drivetrain) {
        return drivetrain.getState().Pose.getY() > 4.00 ? getUpperHome() : getLowerHome();
    }

    /** Whether or not the robot is in the current Alliance's Home. */
    static boolean inHome(Drivetrain drivetrain) {
        double robotXPosition = drivetrain.getState().Pose.getX();

        // If the robot is on the Blue Alliance, return whether it's to the left of the Blue Home Zone threshold.
        // If the robot is on the Red Alliance, return whether it's to the right of the Red Home Zone threshold.
        return Robot.isBlueAlliance ? robotXPosition < Constants.BLUE_X_AXIS_HOME_THRESHOLD
            : Constants.RED_X_AXIS_HOME_THRESHOLD < robotXPosition;
    }
}
