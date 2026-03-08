package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

/**
 * import static frc.robot.Utilities.*;
 */
public interface Utilities {
    /** @return Translation2d of the Hub for the current alliance. */
    static Translation2d getHubPosition() {
        return Robot.isBlueAlliance ? Constants.BLUE_HUB_POSITION : Constants.RED_HUB_POSITION;
    }

    static double getHubDistance(Drivetrain drivetrain) {
        Translation2d hubPosition = getHubPosition();
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        return hubPosition.getDistance(robotPosition);
    }

    static boolean withinShootingDistance(Drivetrain drivetrain) {
        double distance = getHubDistance(drivetrain);
        return Constants.MINIMUM_SHOOTING_DISTANCE < distance && distance < Constants.MAXIMUM_SHOOTING_DISTANCE;
    }

    /** @return Angle in radians from robot to hub. */
    static double getRobotToHubAngle(Drivetrain drivetrain) {
        Translation2d hubPosition = getHubPosition();
        Pose2d currentPose = drivetrain.getState().Pose;
        return Math.atan2((hubPosition.getY() - currentPose.getY()), (hubPosition.getX() - currentPose.getX()));
    }

    static boolean withinShootingAngle(Drivetrain drivetrain) {
        double angleTolerance = Math.toRadians(30); // Placeholder
        double robotRotation = drivetrain.getState().Pose.getRotation().getRadians();
        return Math.abs(MathUtil.angleModulus(getRobotToHubAngle(drivetrain) - robotRotation)) < angleTolerance;
    }

    static boolean inHome(Drivetrain drivetrain) {
        double robotXPosition = drivetrain.getState().Pose.getX();

        // If the Robot is on the Blue Alliance, return whether it's to the left of the Blue Home Zone threshold.
        // If the Robot is on the Red Alliance, return whether it's to the right of the Red Home Zone threshold.
        return Robot.isBlueAlliance ? robotXPosition < Constants.BLUE_X_AXIS_HOME_THRESHOLD
            : robotXPosition > Constants.RED_X_AXIS_HOME_THRESHOLD;
    }

    /**
     * Applies a gear ratio to a TalonFX motor so that getPosition() and getVelocity() automatically return mechanism
     * rotations instead of motor rotor rotations.
     * 
     * @param motor The TalonFX motor to configure.
     * @param gearRatio
     * @return The TalonFX motor that was configured so this can be chained.
     */
    static TalonFX applyGearRatio(TalonFX motor, double gearRatio) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.getConfigurator().refresh(config);
        config.Feedback.SensorToMechanismRatio = gearRatio;
        motor.getConfigurator().apply(config);
        return motor;
    }
}
