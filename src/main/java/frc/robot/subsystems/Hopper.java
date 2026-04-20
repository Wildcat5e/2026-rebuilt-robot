package frc.robot.subsystems;

import static frc.robot.utilities.HardwareUtils.*;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DashboardManager;

public class Hopper extends SubsystemBase implements SysIdCapable {
    private final TalonFX kickerMotor = new TalonFX(14);
    final Slot0Configs kickerFFConfig = new Slot0Configs().withKS(0.).withKV(0.).withKA(0.);

    private final TalonFX conveyorMotor = new TalonFX(15);
    final Slot0Configs conveyerFFConfig = new Slot0Configs().withKS(0.14249).withKV(0.46947).withKA(0.015944);

    // --- SysId Configuration ---
    // ONLY UNCOMMENT THE ROUTINE CREATION LINE FOR THE MOTOR YOU WANT TO CHARACTERIZE.
    // OTHERWISE, YOU RISK DAMAGING YOUR ROBOT.
    // MAKE SURE TO COMMENT OUT THE OTHER ONE BEFORE RUNNING ANY CHARACTERIZATION COMMANDS.

    // --KICKER MOTOR--
    private final SysIdRoutine routine = SysIdCapable.createAngularRoutine(this, kickerMotor, kickerMotor::setVoltage);

    // --KICKER SYSID CONSTANTS--
    // kS: 
    // kV: 
    // kA: 

    // --CONVEYOR MOTOR--
    // private final SysIdRoutine routine =
    //     SysIdCapable.createAngularRoutine(this, conveyorMotor, conveyorMotor::setVoltage);

    // --CONVEYOR SYSID CONSTANTS--
    // kS: 0.14249
    // kV: 0.46947
    // kA: 0.015944

    public Hopper() {
        applyGearRatio(4, conveyorMotor, kickerMotor);
        DashboardManager.setupHopper();
    }

    @Override
    public void periodic() {}

    /** @return The SysIdRoutine used to generate characterization commands. */
    public SysIdRoutine getSysIdRoutine() {
        return routine;
    }

    public void setHopperVoltages(double kickerVoltage, double conveyorVoltage) {
        kickerMotor.setVoltage(kickerVoltage);
        conveyorMotor.setVoltage(conveyorVoltage);
    }


}
