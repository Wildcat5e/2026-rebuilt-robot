package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardManager;
import static frc.robot.Utilities.*;

public class Hopper extends SubsystemBase {
    private final TalonFX leftHopperMotor = new TalonFX(0);
    private final TalonFX rightHopperMotor = new TalonFX(0);
    private final TalonFX kickerMotor = new TalonFX(14);

    public Hopper() {
        applyGearRatio(leftHopperMotor, 0.5);
        applyGearRatio(rightHopperMotor, 0.5);
        applyGearRatio(kickerMotor, 0.5);
        DashboardManager.setupHopper();
    }

    @Override
    public void periodic() {}

    public Command testLeftHopper() {
        return startEnd(() -> leftHopperMotor.setVoltage(3), () -> leftHopperMotor.setVoltage(0));
    }

    public Command testRightHopper() {
        return startEnd(() -> rightHopperMotor.setVoltage(-3), () -> rightHopperMotor.setVoltage(0));
    }

    public Command testBothHoppers() {
        return startEnd(() -> setHopperMotorVoltages(3), () -> setHopperMotorVoltages(0));
    }

    /** Sets both hopper motors to the specified voltage (right voltage negated). */
    private void setHopperMotorVoltages(double volts) {
        leftHopperMotor.setVoltage(volts);
        rightHopperMotor.setVoltage(-volts);
    }

    /** Reads the "Kicker Test Voltage" from SmartDashboard and applies it continuously. */
    public Command testTunableKicker() {
        return runEnd(() -> {
            double targetVoltage = DashboardManager.getKickerTestVoltage();
            kickerMotor.setVoltage(-targetVoltage);
        }, () -> kickerMotor.setVoltage(0));
    }

    public void runFeeder() {
        setHopperMotorVoltages(3);
        kickerMotor.setVoltage(-8);
    }

    public void stopFeeder() {
        setHopperMotorVoltages(0);
        kickerMotor.setVoltage(0);
    }
}
