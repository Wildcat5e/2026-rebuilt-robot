package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardManager;
import static frc.robot.Utilities.*;

public class Hopper extends SubsystemBase {
    private final TalonFX conveyorMotor = new TalonFX(15);
    private final TalonFX kickerMotor = new TalonFX(14);

    public Hopper() {
        applyGearRatio(conveyorMotor, 4);
        applyGearRatio(kickerMotor, 4);
        DashboardManager.setupHopper();
    }

    @Override
    public void periodic() {}

    public Command testConveyor() {
        return startEnd(() -> {
            double targetConveyorVoltage = DashboardManager.getConveyorTestVoltage();
            conveyorMotor.setVoltage(-targetConveyorVoltage);
        }, () -> conveyorMotor.setVoltage(0));
    }

    /** Reads the "Kicker Test Voltage" from SmartDashboard and applies it continuously. */
    public Command testTunableKicker() {
        return runEnd(() -> {
            double targetKickerVoltage = DashboardManager.getKickerTestVoltage();
            kickerMotor.setVoltage(-targetKickerVoltage);
        }, () -> kickerMotor.setVoltage(0));
    }

    public void runHopper() {
        double targetConveyorVoltage = DashboardManager.getConveyorTestVoltage();
        double targetKickerVoltage = DashboardManager.getKickerTestVoltage();
        conveyorMotor.setVoltage(-targetConveyorVoltage);
        kickerMotor.setVoltage(-targetKickerVoltage);
    }

    public void stopHopper() {
        conveyorMotor.setVoltage(0);
        kickerMotor.setVoltage(0);
    }
}
