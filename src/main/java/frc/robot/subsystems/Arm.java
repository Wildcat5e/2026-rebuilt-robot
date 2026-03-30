package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;

/* Interface for ARMS (not elevators) that can utilize Motion Magic control. */
public interface Arm {

    /**
     * Positive velocity indicates that the arm is moving up (stowing).<br>
     * Negative velocity indicates that the arm is moving down (deploying).
     * 
     * @return The current velocity of the arm mechanism.
     */
    default double getArmVelocity() {
        return getMotor().getVelocity().getValueAsDouble();
    }

    /** @return 0 if arm is not moving. -1 if arm is moving down (deploying). 1 if arm is moving up (stowing). */
    default int getArmMovementStatus() {
        if (!isArmMoving()) return 0;

        double velocity = getArmVelocity();

        // Math.signum returns -1.0 for negative, 0.0 for zero, and 1.0 for positive numbers.
        int status = (int) Math.signum(velocity);

        if (Math.abs(status) > 1) {
            throw new IllegalStateException(
                "Invalid movement status calculated: " + status + ". Value must be -1, 0, or 1.");
        }

        return status;
    }

    default boolean isArmMoving() {
        return Math.abs(getArmVelocity()) > 0.1;
    }

    /** @return The motor controller moving the arm. */
    TalonFX getMotor();

    /** @return The subsystem itself (usually 'this') for command requirements. */
    Subsystem getAssociatedSubsystem();
}
