package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.function.DoubleConsumer;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
     * Applies Motion Magic configuration and PID/FeedForward gains to a TalonFX motor.
     */
    static void configureMotionMagic(TalonFX motor, double kP, double kI, double kD, double kS, double kV, double kG,
        double cruiseVelocity, double acceleration, double jerk) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.getConfigurator().refresh(config);

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kG = kG;

        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity; // rps
        config.MotionMagic.MotionMagicAcceleration = acceleration; // rps/s
        config.MotionMagic.MotionMagicJerk = jerk; // Set to >0 for S-Curve smoothing
    }

    /**
     * Creates a SysIdRoutine for a linear mechanism using TalonFX motors.
     * 
     * @param subsystem The subsystem being characterized.
     * @param primaryMotor The motor used for logging telemetry (if using >1 motors, pick one with +voltage).
     * @param applyVoltage A consumer that applies the generated voltage to the motor(s).
     * @param distancePerRotation The conversion factor from motor rotations to meters.
     */
    static SysIdRoutine createLinearRoutine(Subsystem subsystem, TalonFX primaryMotor, DoubleConsumer applyVoltage,
        double distancePerRotation) {
        return new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(voltage -> applyVoltage.accept(voltage.in(Volts)), log -> {
                log.motor(subsystem.getName() + "-motor(s)").voltage(primaryMotor.getMotorVoltage().getValue())
                    .linearPosition(Distance
                        .ofRelativeUnits(primaryMotor.getPosition().getValueAsDouble() * distancePerRotation, Meters))
                    .linearVelocity(LinearVelocity.ofRelativeUnits(
                        primaryMotor.getVelocity().getValueAsDouble() * distancePerRotation, MetersPerSecond));
            }, subsystem));
    }

    /** Returns a command to run a quasistatic forward test for the given routine. */
    static Command sysIdQuasistaticForward(SysIdRoutine routine) {
        return routine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    /** Returns a command to run a quasistatic reverse test for the given routine. */
    static Command sysIdQuasistaticReverse(SysIdRoutine routine) {
        return routine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    /** Returns a command to run a dynamic forward test for the given routine. */
    static Command sysIdDynamicForward(SysIdRoutine routine) {
        return routine.dynamic(SysIdRoutine.Direction.kForward);
    }

    /** Returns a command to run a dynamic reverse test for the given routine. */
    static Command sysIdDynamicReverse(SysIdRoutine routine) {
        return routine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
