package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;

public interface Constants {
    /** kSpeedAt12Volts desired top speed in m/s */
    double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /** 540 deg/sec max angular velocity in rad/sec */
    double MAX_ANGULAR_SPEED = DegreesPerSecond.of(540).in(RadiansPerSecond);
    /** 720 deg/sec^2 max angular acceleration in rad/sec^2 */
    double MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(720).in(RadiansPerSecondPerSecond);

    // These values will need to be adjusted based on the actual robot's shooting distance capabilities
    double MINIMUM_SHOOTING_DISTANCE = 1.0; // meters
    double MAXIMUM_SHOOTING_DISTANCE = 3.0; // meters

    /** Field layout. Change depending on whether field uses AndyMark or Welded specs */
    AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    Translation2d BLUE_HUB_POSITION = new Translation2d(4.625, 4.03);
    Translation2d RED_HUB_POSITION = new Translation2d(11.915, 4.03);
    double BLUE_X_AXIS_HOME_THRESHOLD = 4.0;
    double RED_X_AXIS_HOME_THRESHOLD = 12.54;
}
