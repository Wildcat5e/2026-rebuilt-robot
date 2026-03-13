package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MotionMagicCapable {

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
