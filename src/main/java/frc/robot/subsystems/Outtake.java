package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {

    private final TalonFX leftHopperMotor = new TalonFX(0);
    private final TalonFX rightHopperMotor = new TalonFX(0);
    private final TalonFX kickerMotor = new TalonFX(0);
    private final TalonFX flywheelMotor = new TalonFX(0);

    /** Creates a new Outtake. */
    public Outtake() {}

    @Override
    public void periodic() {}

    public Command testLeftHopper() {
        // might have to change voltage signs, left and right
        // motor should spin in different directions
        return startEnd(() -> leftHopperMotor.setVoltage(3), () -> leftHopperMotor.setVoltage(0));
    }

    public Command testRightHopper() {
        return startEnd(() -> rightHopperMotor.setVoltage(-3), () -> rightHopperMotor.setVoltage(0));
    }

    public Command testBothHoppers() {
        return startEnd(() -> {
            leftHopperMotor.setVoltage(3);
            rightHopperMotor.setVoltage(-3);
        }, () -> {
            leftHopperMotor.setVoltage(0);
            rightHopperMotor.setVoltage(0);
        });
    }


    public Command spinKicker() {
        return startEnd(() -> kickerMotor.setVoltage(3), () -> kickerMotor.setVoltage(0));
    }

    public Command spinFlywheel() {
        return startEnd(() -> flywheelMotor.setVoltage(3), () -> flywheelMotor.setVoltage(0));
    }

    // activates flywheel, command ends after certain amount of motor ticks
    // but does not set voltage of flywheel to 0
    public Command startFlywheel() {
        return runOnce(() -> flywheelMotor.setVoltage(3));
    }

    public Command endFlywheel() {
        return startEnd(() -> flywheelMotor.setVoltage(3), () -> flywheelMotor.setVoltage(0));
    }


    // final implementation should be a while true
    public Command shootFuel() {
        return new SequentialCommandGroup(new ParallelRaceGroup(startFlywheel(),
            // could do something where you check the amount of motor ticks that have passed
            // to infer speed of flywheel instead of waiting time
            Commands.waitSeconds(1)), new ParallelRaceGroup(testBothHoppers(), spinKicker(), endFlywheel()));
    }
}
