package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface SysIdCapable {

    /** @return The active SysIdRoutine used to generate characterization commands. */
    SysIdRoutine getSysIdRoutine();

    /**
     * Creates a SysIdRoutine for an angular mechanism using TalonFX motors.
     * 
     * @param subsystem The subsystem being characterized.
     * 
     * @param primaryMotor The motor used for logging telemetry (if using >1 motors, pick one with +voltage).
     * @param applyVoltage A consumer that applies the generated voltage to the motor(s).
     */
    static SysIdRoutine createAngularRoutine(Subsystem subsystem, TalonFX primaryMotor, DoubleConsumer applyVoltage) {
        return new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(voltage -> applyVoltage.accept(voltage.in(Volts)), log -> {
                log.motor(subsystem.getName() + "-motor(s)").voltage(primaryMotor.getMotorVoltage().getValue())
                    .angularPosition(Angle.ofRelativeUnits(primaryMotor.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(AngularVelocity.ofRelativeUnits(primaryMotor.getVelocity().getValueAsDouble(),
                        RotationsPerSecond));
            }, subsystem));
    }

    default Command sysIdQuasistaticForward() {
        return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward);
    }

    default Command sysIdQuasistaticReverse() {
        return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse);
    }

    default Command sysIdDynamicForward() {
        return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward);
    }

    default Command sysIdDynamicReverse() {
        return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse);
    }
}
