package frc.robot.utilities;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

/**
 * import static frc.robot.utilities.HardwareUtils.*;
 */
public interface HardwareUtils {
    /** DO NOT MODIFY DIRECTLY! */
    class Requests {
        private static final VelocityVoltage VEL_REQUEST = new VelocityVoltage(0).withSlot(0);
    }

    /**
     * Applies a gear ratio to a TalonFX motor so that getPosition(), getVelocity(), etc., automatically return
     * mechanism rotations instead of motor revolutions.
     * 
     * @param gearRatio The gear ratio to apply.
     * @param motors The TalonFX motor(s) to configure.
     */
    static void applyGearRatio(double gearRatio, TalonFX... motors) {
        var config = new TalonFXConfiguration();
        for (var motor : motors) {
            motor.getConfigurator().refresh(config);
            config.Feedback.SensorToMechanismRatio = gearRatio;
            motor.getConfigurator().apply(config);
        }
    }

    /** Sets the motor to Clockwise_Positive. Motor defaults to CounterClockwise_Positive. */
    static void setMotorClockwisePositive(TalonFX... motors) {
        var config = new TalonFXConfiguration();
        for (var motor : motors) {
            motor.getConfigurator().refresh(config);
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            motor.getConfigurator().apply(config);
        }
    }

    /**
     * Sets the motor to CounterClockwise_Positive. This is what the motor defaults to, so you should only use this if
     * you are reverting {@link #setMotorClockwisePositive}.
     */
    static void setMotorCounterClockwisePositive(TalonFX... motors) {
        var config = new TalonFXConfiguration();
        for (var motor : motors) {
            motor.getConfigurator().refresh(config);
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            motor.getConfigurator().apply(config);
        }
    }

    static void applyFeedforward(Slot0Configs feedforwardConfig, TalonFX... motors) {
        var config = new TalonFXConfiguration();
        for (var motor : motors) {
            motor.getConfigurator().refresh(config);
            config.Slot0 = feedforwardConfig;
            motor.getConfigurator().apply(config);
        }
    }

    /**
     * Sets velocity for motor using motor controller for feedforward (VelocityVoltage). Requires kS in volts in Slot0.
     */
    static void setVelocity(double velocity, TalonFX... motors) {
        Requests.VEL_REQUEST.Velocity = velocity;
        for (var motor : motors) {
            motor.setControl(Requests.VEL_REQUEST);
        }
    }
}
