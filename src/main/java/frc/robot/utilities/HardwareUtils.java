package frc.robot.utilities;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * import static frc.robot.utilities.HardwareUtils.*;
 */
public interface HardwareUtils {
    /**
     * Applies a gear ratio to a TalonFX motor so that getPosition(), getVelocity(), etc., automatically return
     * mechanism rotations instead of motor revolutions.
     * 
     * @param motor The TalonFX motor to configure.
     * @param gearRatio The gear ratio to apply.
     * @return The TalonFX motor that was configured so this can be chained.
     */
    static TalonFX applyGearRatio(TalonFX motor, double gearRatio) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.getConfigurator().refresh(config);
        config.Feedback.SensorToMechanismRatio = gearRatio;
        motor.getConfigurator().apply(config);
        return motor;
    }
}
