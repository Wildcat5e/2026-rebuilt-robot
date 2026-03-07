package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Utilities.*;

public class Hopper extends SubsystemBase {
    private final TalonFX leftHopperMotor = new TalonFX(0);
    private final TalonFX rightHopperMotor = new TalonFX(0);
    private final TalonFX kickerMotor = new TalonFX(0);

    public Hopper() {
        applyGearRatio(leftHopperMotor, 0.5);
        applyGearRatio(rightHopperMotor, 0.5);
        applyGearRatio(kickerMotor, 0.5);
    }

    @Override
    public void periodic() {}

    public Command testLeftHopper() {
        // Voltage can be negated to swap direction
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

    public Command testSpinKicker() {
        return startEnd(() -> kickerMotor.setVoltage(3), () -> kickerMotor.setVoltage(0));
    }

    public void runFeeder() {
        leftHopperMotor.setVoltage(3);
        rightHopperMotor.setVoltage(-3);
        kickerMotor.setVoltage(3);
    }

    public void stopFeeder() {
        leftHopperMotor.setVoltage(0);
        rightHopperMotor.setVoltage(0);
        kickerMotor.setVoltage(0);
    }
}
