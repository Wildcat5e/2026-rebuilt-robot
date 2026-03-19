package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DashboardManager;
import static frc.robot.Utilities.*;

public class Hopper extends SubsystemBase implements SysIdCapable {
    private final TalonFX kickerMotor = new TalonFX(14);
    private final double KICKER_RADIUS = 0.03175 / 2;
    private final double KICKER_CIRCUMFERENCE = 2 * Math.PI * KICKER_RADIUS;

    private final TalonFX conveyorMotor = new TalonFX(15);
    private final double CONVEYOR_RADIUS = 0.028575 / 2;
    private final double CONVEYOR_CIRCUMFERENCE = 2 * Math.PI * CONVEYOR_RADIUS;

    // --- SysId Configuration ---
    // ONLY UNCOMMENT THE ROUTINE CREATION LINE FOR THE MOTOR YOU WANT TO CHARACTERIZE.
    // OTHERWISE, YOU RISK DAMAGING YOUR ROBOT.
    // MAKE SURE TO COMMENT OUT THE OTHER TWO BEFORE RUNNING ANY CHARACTERIZATION COMMANDS.

    // --KICKER MOTOR--
    private final SysIdRoutine routine =
        createLinearRoutine(this, kickerMotor, kickerMotor::setVoltage, KICKER_CIRCUMFERENCE);

    // --CONVEYOR MOTOR--
    // private final SysIdRoutine routine =
    //     createLinearRoutine(this, conveyorMotor, conveyorMotor::setVoltage, CONVEYOR_CIRCUMFERENCE);

    /** @return The SysIdRoutine used to generate characterization commands. */
    public SysIdRoutine getSysIdRoutine() {
        return routine;
    }

    public Hopper() {
        applyGearRatio(kickerMotor, 4);
        applyGearRatio(conveyorMotor, 4);
        DashboardManager.setupHopper();
    }

    @Override
    public void periodic() {}

    public Command reverseConveyor() {
        return startEnd(() -> conveyorMotor.setVoltage(3), () -> conveyorMotor.setVoltage(0))
            .withName("Reverse Conveyor");
    }

    /** Reads the "Kicker Test Voltage" from SmartDashboard and applies it continuously. */
    public Command testTunableKicker() {
        return runEnd(() -> {
            double targetKickerVoltage = DashboardManager.getKickerTestVoltage();
            kickerMotor.setVoltage(-targetKickerVoltage);
        }, () -> kickerMotor.setVoltage(0));
    }

    public Command testConveyor() {
        return startEnd(() -> {
            double targetConveyorVoltage = DashboardManager.getConveyorTestVoltage();
            conveyorMotor.setVoltage(-targetConveyorVoltage);
        }, () -> conveyorMotor.setVoltage(0));
    }

    public Command reverseKicker() {
        return startEnd(() -> kickerMotor.setVoltage(3), () -> kickerMotor.setVoltage(0)).withName("Reverse Kicker");
    }

    public void runHopper() {
        double targetConveyorVoltage = DashboardManager.getConveyorTestVoltage();
        double targetKickerVoltage = DashboardManager.getKickerTestVoltage();
        conveyorMotor.setVoltage(-targetConveyorVoltage);
        kickerMotor.setVoltage(-targetKickerVoltage);
    }

    public void stopConveyor() {
        conveyorMotor.setVoltage(0);
    }
}
