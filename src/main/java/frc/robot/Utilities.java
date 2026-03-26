package frc.robot;

import java.util.Optional;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Drivetrain;

/**
 * import static frc.robot.Utilities.*;
 */
public interface Utilities {
    public enum MatchPhase {
        TRANSITION(130.0), SHIFT_1(105.0), SHIFT_2(80.0), SHIFT_3(55.0), SHIFT_4(30.0), ENDGAME(0.0);

        /* Measured in seconds remaining in the match. */
        public final double endTime;

        MatchPhase(double endTime) {
            this.endTime = endTime;
        }

        public static MatchPhase getPhase(double matchTime) {
            if (matchTime > 130) return TRANSITION;
            if (matchTime > 105) return SHIFT_1;
            if (matchTime > 80) return SHIFT_2;
            if (matchTime > 55) return SHIFT_3;
            if (matchTime > 30) return SHIFT_4;
            return ENDGAME;
        }
    }

    static boolean isHubActive() {
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as it's likely early in teleop.
        if (gameData == null || gameData.isEmpty()) {
            DriverStation.reportWarning("Game Data is empty.", false);
            return true;
        }

        char firstChar = gameData.charAt(0);
        if (firstChar != 'R' && firstChar != 'B') {
            DriverStation.reportWarning("Game Data is invalid: " + gameData, false);
            return true;
        }

        // Shift 1 is active for Blue if Red won auto ('R'), or for Red if Blue won auto ('B')
        boolean redInactiveFirst = (firstChar == 'R');
        boolean shift1Active = Robot.isBlueAlliance ? redInactiveFirst : !redInactiveFirst;

        MatchPhase currentPhase = MatchPhase.getPhase(DriverStation.getMatchTime());

        return switch (currentPhase) {
            case TRANSITION -> true;
            case SHIFT_1 -> shift1Active;
            case SHIFT_2 -> !shift1Active;
            case SHIFT_3 -> shift1Active;
            case SHIFT_4 -> !shift1Active;
            case ENDGAME -> true;
        };
    }

    static double getHubShiftTimeRemaining() {
        if (!DriverStation.isTeleopEnabled()) {
            return 0.0;
        }

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) {
            return 0.0;
        }

        MatchPhase currentPhase = MatchPhase.getPhase(matchTime);
        return matchTime - currentPhase.endTime;
    }

    /** @return Translation2d of the Hub for the current alliance. */
    static Translation2d getHubPosition() {
        return Robot.isBlueAlliance ? Constants.BLUE_HUB_POSITION : Constants.RED_HUB_POSITION;
    }

    static double getTargetDistance(Drivetrain drivetrain, Translation2d target) {
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        return target.getDistance(robotPosition);
    }

    static boolean withinShootingDistance(Drivetrain drivetrain, Translation2d target) {
        double distance = getTargetDistance(drivetrain, target);
        return Constants.MINIMUM_SHOOTING_DISTANCE < distance && distance < Constants.MAXIMUM_SHOOTING_DISTANCE;
    }

    /** @return Angle in radians from robot to target. */
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
        return Robot.isBlueAlliance ? robotXPosition < Constants.BLUE_X_AXIS_HOME_THRESHOLD
            : robotXPosition > Constants.RED_X_AXIS_HOME_THRESHOLD;
    }

    /** Rounds a double to a specified number of decimal places */
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

    /**
     * @return Returns a translation2d of the spot where the robot should shoot when shooting from neutral zone into
     *         home.
     */
    static Translation2d getHomeTarget(Drivetrain drivetrain) {
        // In aim handler, we have a dead zone between y = 4.25 and y = 3.75, so the robot doesn't rotate in that space.
        // But this method is to be used for shootingCalculator and finding the speed to set the flywheel, so I
        // don't believe we should have a deadzone for not being able to shoot fuel, as it could be catastrophic
        // if our pose is incorrect.
        if (drivetrain.getState().Pose.getY() > 4.00) {
            return getUpperHome();
        } else {
            return getLowerHome();
        }
    }

    /** @return Translation2d of the Upper Home for the current Alliance. */
    static Translation2d getUpperHome() {
        return Robot.isBlueAlliance ? Constants.UPPER_HOMES.get(0) : Constants.UPPER_HOMES.get(1);
    }

    /** @return Translation2d of the Lower Home for the current Alliance. */
    static Translation2d getLowerHome() {
        return Robot.isBlueAlliance ? Constants.LOWER_HOMES.get(0) : Constants.LOWER_HOMES.get(1);
    }
}
