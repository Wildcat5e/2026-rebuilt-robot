package frc.robot;

import static edu.wpi.first.units.Units.*;
import frc.robot.generated.TunerConstants;

public interface Constants {
    /** kSpeedAt12Volts desired top speed */
    double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** 3/4 revs per sec max angular velocity in radians per second */
    double MAX_ANGULAR_SPEED = DegreesPerSecond.of(540).in(RadiansPerSecond);
    double MAX_ANGULAR_ACCEL = DegreesPerSecond.of(720).in(RadiansPerSecond);

}
