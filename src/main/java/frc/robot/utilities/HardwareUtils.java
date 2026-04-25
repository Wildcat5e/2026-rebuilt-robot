package frc.robot.utilities;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * import static frc.robot.utilities.HardwareUtils.*;
 */
public interface HardwareUtils {
    /**
     * Checks the Driver Station to ensure the correct controllers are plugged into the expected ports. Expects an Xbox
     * Controller on Port 0 and a CircuitPython Macropad (GenericHID) on Port 4.
     */
    static boolean checkDriverStationPorts() {
        boolean allOkay = true;

        if (!DriverStation.isJoystickConnected(0)) {
            DriverStation.reportWarning("Port 0: No controller connected! Expected an Xbox Controller.", false);
            allOkay = false;
        } else if (!DriverStation.getJoystickIsXbox(0)) {
            DriverStation.reportWarning("Port 0: Connected device is NOT an Xbox Controller!", false);
            allOkay = false;
        }

        if (!DriverStation.isJoystickConnected(4)) {
            DriverStation.reportWarning("Port 4: No device connected! Expected a CircuitPython Macropad.", false);
            allOkay = false;
        } else {
            String joystickName = DriverStation.getJoystickName(4);
            // We check for the device name defined in boot.py
            if (joystickName != null && !joystickName.toLowerCase().contains("macropad")
                && !joystickName.toLowerCase().contains("circuitpython")) {
                DriverStation.reportWarning("Port 4: Connected device is '" + joystickName + "'. Expected a Macropad.",
                    false);
                allOkay = false;
            }
        }

        return allOkay;
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
        var request = new VelocityVoltage(velocity).withSlot(0);
        for (var motor : motors) {
            motor.setControl(request);
        }
    }
}
