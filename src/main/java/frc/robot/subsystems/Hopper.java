package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardManager;
import static frc.robot.Utilities.*;

public class Hopper extends SubsystemBase {
    private final TalonFX hopperMotor = new TalonFX(15);
    private final TalonFX kickerMotor = new TalonFX(14);

    public Hopper() {
        applyGearRatio(hopperMotor, 4);
        applyGearRatio(kickerMotor, 4);
        DashboardManager.setupHopper();
    }

    @Override
    public void periodic() {}

    public Command testHopper() {
        return startEnd(() -> hopperMotor.setVoltage(3), () -> hopperMotor.setVoltage(0));
    }

    /** Reads the "Kicker Test Voltage" from SmartDashboard and applies it continuously. */
    public Command testTunableKicker() {
        return runEnd(() -> {
            double targetVoltage = DashboardManager.getKickerTestVoltage();
            kickerMotor.setVoltage(-targetVoltage);
        }, () -> kickerMotor.setVoltage(0));
    }

    public void runFeeder() {
        hopperMotor.setVoltage(3);
        kickerMotor.setVoltage(-8);
    }

    public void stopFeeder() {
        hopperMotor.setVoltage(0);
        kickerMotor.setVoltage(0);
    }
}
