package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static java.util.Arrays.stream;

/**
 * import static frc.robot.Utilities.*;
 */
public interface Utilities {
    /**
     * @return Translation2d of the Hub for the current alliance.
     */
    static Translation2d getHubPosition() {
        return isBlueAlliance() ? Constants.BLUE_HUB_POSITION : Constants.RED_HUB_POSITION;
    }

    static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == Blue;
    }

    static double getTargetDistance(Drivetrain drivetrain, Translation2d target) {
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        return target.getDistance(robotPosition);
    }

    static boolean withinShootingDistance(Drivetrain drivetrain, Translation2d target) {
        double distance = getTargetDistance(drivetrain, target);
        return Constants.MINIMUM_SHOOTING_DISTANCE < distance && distance < Constants.MAXIMUM_SHOOTING_DISTANCE;
    }

    /**
     * @return Angle in radians from robot to target.
     */
    static double getRobotToTargetAngle(Drivetrain drivetrain, Translation2d target) {
        Pose2d currentPose = drivetrain.getState().Pose;
        return Math.atan2((target.getY() - currentPose.getY()), (target.getX() - currentPose.getX()));
    }

    /**
     * @deprecated This needs to be replaced/removed because of {@link frc.robot.subsystems.ShootingCalculator } and
     *             because reversed shooter.
     */
    static boolean withinShootingAngle(Drivetrain drivetrain, Translation2d target) {
        double robotRotation = drivetrain.getState().Pose.getRotation().getRadians();
        double angleDiff = MathUtil.angleModulus(getRobotToTargetAngle(drivetrain, target) - robotRotation);
        return Math.abs(angleDiff) < Math.toRadians(30); // Split out to constant if we make a new one of this function.
    }

    static boolean inHome(Drivetrain drivetrain) {
        double robotXPosition = drivetrain.getState().Pose.getX();

        // If the Robot is on the Blue Alliance, return whether it's to the left of the Blue Home Zone threshold.
        // If the Robot is on the Red Alliance, return whether it's to the right of the Red Home Zone threshold.
        return isBlueAlliance() ? robotXPosition < Constants.BLUE_X_AXIS_HOME_THRESHOLD
            : robotXPosition > Constants.RED_X_AXIS_HOME_THRESHOLD;
    }

    /**
     * Rounds a double to a specified number of decimal places
     */
    static double round(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
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

    static Translation2d getUpperHome() {
        return isBlueAlliance() ? Constants.BLUE_UPPER_HOME : Constants.RED_UPPER_HOME;
    }

    static Translation2d getLowerHome() {
        return isBlueAlliance() ? Constants.BLUE_LOWER_HOME : Constants.RED_LOWER_HOME;
    }

    static void debug(String... messages) {
        ifTest(() -> stream(messages).forEach(System.out::println));
    }

    static void ifTest(Runnable runnable) {
        if (DriverStation.isTest()) {
            runnable.run();
        }
    }
}
