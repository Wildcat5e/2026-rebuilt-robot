package frc.robot.subsystems;

import static frc.robot.utilities.HardwareUtils.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardManager;

public class Hopper extends SubsystemBase {
    private final TalonFX conveyorMotor = new TalonFX(15);
    private final TalonFX kickerMotor = new TalonFX(14);

    public Hopper() {
        applyGearRatio(4, conveyorMotor, kickerMotor);
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

    public Command reverseConveyor() {
        return startEnd(() -> conveyorMotor.setVoltage(12), () -> conveyorMotor.setVoltage(0))
            .withName("Reverse Conveyor");
    }

    /** Reads the "Kicker Test Voltage" from SmartDashboard and applies it continuously. */
    public Command testTunableKicker() {
        return runEnd(() -> {
            double targetKickerVoltage = DashboardManager.getKickerTestVoltage();
            kickerMotor.setVoltage(-targetKickerVoltage);
        }, () -> kickerMotor.setVoltage(0));
    }

    public Command reverseKicker() {
        return startEnd(() -> kickerMotor.setVoltage(12), () -> kickerMotor.setVoltage(0)).withName("Reverse Kicker");
    }

    public void runHopper() {
        double targetConveyorVoltage = DashboardManager.getConveyorTestVoltage();
        double targetKickerVoltage = DashboardManager.getKickerTestVoltage();
        conveyorMotor.setVoltage(-targetConveyorVoltage);
        kickerMotor.setVoltage(-targetKickerVoltage);
    }

    public Command runHopperCommand() {
        return startEnd(() -> {
            double targetConveyorVoltage = DashboardManager.getConveyorTestVoltage();
            double targetKickerVoltage = DashboardManager.getKickerTestVoltage();
            conveyorMotor.setVoltage(-targetConveyorVoltage);
            kickerMotor.setVoltage(-targetKickerVoltage);
        }, () -> {
            conveyorMotor.setVoltage(0);
            kickerMotor.setVoltage(0);
        });
    }

    public void stopHopper() {
        conveyorMotor.setVoltage(0);
        kickerMotor.setVoltage(0);
    }
}
