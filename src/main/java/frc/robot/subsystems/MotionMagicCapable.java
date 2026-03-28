package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

/* Interface for ARMS (not elevators) that can utilize Motion Magic control. */
public interface MotionMagicCapable {

    /** Applies Motion Magic configuration and PID/FeedForward gains to a TalonFX motor. */
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
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity; // Max speed in rps
        config.MotionMagic.MotionMagicAcceleration = acceleration; // Max accel in rps/s
        config.MotionMagic.MotionMagicJerk = jerk; // Max jerk in rps/s^2. Set to >0 for S-Curve smoothing

        motor.getConfigurator().apply(config);
    }

    /** Zero the position of the motor. */
    default Command zeroPosition() {
        return Commands.runOnce(() -> getMotionMagicMotor().setPosition(0));
    }

    /** @return The current position of the Motion Magic mechanism. */
    default double getMechanismPosition() {
        return getMotionMagicMotor().getPosition().getValueAsDouble();
    }

    /** @return The motor controller running the Motion Magic profile. */
    TalonFX getMotionMagicMotor();

    /** @return The control request object used to send targets to the motor. */
    MotionMagicVoltage getMotionMagicRequest();

    /** @return The subsystem itself (usually 'this') for command requirements. */
    Subsystem getAssociatedSubsystem();

    /**
     * Generates a WPILib Command to smoothly move the mechanism to a target position.
     * 
     * @param targetPosition The desired position (in mechanism rotations).
     * @param tolerance The error tolerance to determine when the move is finished.
     * @return A command that runs until the target is reached within the tolerance.
     */
    default Command moveToPosition(double targetPosition, double tolerance) {
        return Commands
            .run(() -> getMotionMagicMotor().setControl(getMotionMagicRequest().withPosition(targetPosition)),
                getAssociatedSubsystem())
            .until(() -> {
                double currentPosition = getMotionMagicMotor().getPosition().getValueAsDouble();
                return Math.abs(currentPosition - targetPosition) <= tolerance;
            });
    }
}
