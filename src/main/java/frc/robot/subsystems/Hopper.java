package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final TalonFX leftHopperMotor = new TalonFX(0);
    private final TalonFX rightHopperMotor = new TalonFX(0);
    private final TalonFX kickerMotor = new TalonFX(14);

    public Hopper() {
        SmartDashboard.putNumber("Kicker Test Voltage", 8);
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
        return startEnd(() -> kickerMotor.setVoltage(-8), () -> kickerMotor.setVoltage(0));
    }

    public Command testTunableKicker() {
        return runEnd(() -> {
            double targetVoltage = SmartDashboard.getNumber("Kicker Test Voltage", 0.0);
            kickerMotor.setVoltage(-targetVoltage);
        }, () -> kickerMotor.setVoltage(0));
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
