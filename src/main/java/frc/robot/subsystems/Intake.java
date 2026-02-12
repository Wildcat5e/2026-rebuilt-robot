package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    /** place holder */
    double GEAR_RATIO = 0.5;
    /** CHANGE MOTOR ID OBVIOUSLY, conveyor is motor that connects to actual wheels to intake fuel into storage */
    private final TalonFX conveyorMotor = new TalonFX(0);
    /** extender motor is motor that extends intake system outside of bumper */
    private final TalonFX extenderMotor = new TalonFX(0);

    /** Creates a new Intake. */
    public Intake() {}

    @Override
    public void periodic() {}

    public Command dropArm() {
        return new FunctionalCommand(
            // initialize
            // if this doesnt work, might have to put set voltage inside execute?
            () -> extenderMotor.setVoltage(3),
            // execute
            () -> {},
            // end
            interrupted -> extenderMotor.setVoltage(0),
            // is finished
            () -> {
                // check sign
                return getExtenderPosition() >= .3;
            },
            // requirements (what is this)
            this);
    }

    /**
     * TURN ON INTAKE TO TAKE IN FUEL COMMAND <br>
     * currently plan to bind to a while true button, but may be easier to have it on a toggle or have it to run the
     * entire match
     */
    public Command testFuelIntake() {
        return startEnd(
            // start
            () -> conveyorMotor.setVoltage(4),
            // end
            () -> conveyorMotor.setVoltage(0));
    }

    // untested clockwise/ccw
    public Command turnArmClockwise() {
        return startEnd(
            // start, runs once
            () -> extenderMotor.setVoltage(3),
            // end, runs once
            () -> extenderMotor.setVoltage(0));
    }

    public Command turnArmCounterClockwise() {
        return startEnd(
            // start, runs once
            () -> extenderMotor.setVoltage(-3),
            // end, runs once
            () -> extenderMotor.setVoltage(0));
    }

    /** In rotations of arm */
    public double getExtenderPosition() {
        return extenderMotor.getPosition().getValueAsDouble() * GEAR_RATIO;
    }
}
