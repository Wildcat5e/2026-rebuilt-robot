package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardManager;

import static frc.robot.Utilities.applyGearRatio;

public class Hopper extends SubsystemBase {
    private final TalonFX conveyorMotor = new TalonFX(15);
    private final TalonFX kickerMotor = new TalonFX(14);

    public Hopper() {
        applyGearRatio(conveyorMotor, 4);
        applyGearRatio(kickerMotor, 4);
        DashboardManager.setupHopper();
    }

    public void start() {
        double targetConveyorVoltage = DashboardManager.getConveyorTestVoltage();
        double targetKickerVoltage = DashboardManager.getKickerTestVoltage();
        conveyorMotor.setVoltage(-targetConveyorVoltage);
        kickerMotor.setVoltage(-targetKickerVoltage);
    }

    public void stop() {
        conveyorMotor.setVoltage(0);
        kickerMotor.setVoltage(0);
    }

    public Command testConveyor() {
        return startEnd(() -> conveyorMotor.setVoltage(-DashboardManager.getConveyorTestVoltage()),
            () -> conveyorMotor.setVoltage(0)).withName("Test Conveyor");
    }

    public Command reverseConveyor() {
        return startEnd(() -> conveyorMotor.setVoltage(3), () -> conveyorMotor.setVoltage(0))
            .withName("Reverse Conveyor");
    }

    /**
     * Reads the "Kicker Test Voltage" from SmartDashboard and applies it continuously.
     */
    public Command testTunableKicker() {
        return runEnd(() -> kickerMotor.setVoltage(-DashboardManager.getKickerTestVoltage()),
            () -> kickerMotor.setVoltage(0)).withName("Test Tunable Kicker");
    }

    public Command reverseKicker() {
        return startEnd(() -> kickerMotor.setVoltage(3), () -> kickerMotor.setVoltage(0)).withName("Reverse Kicker");
    }

    public Command runHopper() {
        return startEnd(this::start, this::stop).withName("Run Hopper");
    }
}
