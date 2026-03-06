package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static final double GEAR_RATIO = 0.5; // Placeholder
    /** Motor that connects to actual wheels to intake fuel into storage. */
    private final TalonFX conveyorMotor = new TalonFX(0);
    /** Motor that extends intake system outside of bumper. */
    private final TalonFX extenderMotor = new TalonFX(0);

    public Intake() {}

    @Override
    public void periodic() {}

    public Command dropArm() {
        return new FunctionalCommand(
            // --onInit--
            () -> extenderMotor.setVoltage(3), // If this doesn't work, might have to put setVoltage() inside execute

            // --onExecute--
            () -> {},

            // --onEnd--
            interrupted -> extenderMotor.setVoltage(0),

            // --isFinished--
            () -> {
                return getExtenderPosition() >= .3; // Check sign
            },

            // --requirements--
            this); // Pass in Intake
    }

    /**
     * TURN ON INTAKE TO TAKE IN FUEL COMMAND <br>
     * Currently plan to bind to a while true button, but may be easier to have it run on toggle or for the entire match
     */
    public Command testFuelIntake() {
        return startEnd(
            // --onStart--
            () -> conveyorMotor.setVoltage(4),
            // --onEnd--
            () -> conveyorMotor.setVoltage(0));
    }

    // Untested
    public Command turnArmClockwise() {
        return startEnd(
            // --onStart--
            () -> extenderMotor.setVoltage(3),
            // --onEnd--
            () -> extenderMotor.setVoltage(0));
    }

    // Untested
    public Command turnArmCounterClockwise() {
        return startEnd(
            // --onStart--
            () -> extenderMotor.setVoltage(-3),
            // --onEnd--
            () -> extenderMotor.setVoltage(0));
    }

    /** @return Extender motor's position in rotations */
    public double getExtenderPosition() {
        return extenderMotor.getPosition().getValueAsDouble() * GEAR_RATIO;
    }
}
