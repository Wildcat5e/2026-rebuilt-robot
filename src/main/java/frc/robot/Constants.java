package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.List;
import java.util.Map;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.generated.TunerConstants;

public interface Constants {
    /** kSpeedAt12Volts desired top speed in m/s */
    double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /** 540 deg/sec max angular velocity in rad/sec */
    double MAX_ANGULAR_SPEED = DegreesPerSecond.of(540).in(RadiansPerSecond);
    /** 720 deg/sec^2 max angular acceleration in rad/sec^2 */
    double MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(720).in(RadiansPerSecondPerSecond);

    /** The angle of the shooter's hood, as measured from the horizontal at the point where the ball leaves the hood. */
    double HOOD_ANGLE_RADIANS = Math.toRadians(64);

    // These values will need to be adjusted based on the actual robot's shooting distance capabilities
    double MINIMUM_SHOOTING_DISTANCE = 1.0; // meters
    double MAXIMUM_SHOOTING_DISTANCE = 3.0; // meters

    /** Field layout. Change depending on whether field uses AndyMark or Welded specs */
    AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    Translation2d BLUE_HUB_POSITION = new Translation2d(4.625, 4.03);
    Translation2d RED_HUB_POSITION = new Translation2d(11.915, 4.03);

    /** x-coordinate Threshold for Blue Alliance Home Zone */
    double BLUE_X_AXIS_HOME_THRESHOLD = 4.0;
    /** x-coordinate Threshold for Red Alliance Home Zone */
    double RED_X_AXIS_HOME_THRESHOLD = 12.54;

    /** Index 0 is Blue. Upper homes are Blue Left and Red Right. */
    List<Translation2d> UPPER_HOMES = List.of(new Translation2d(2, 6.25), new Translation2d(14.5, 6.25));
    /** Index 0 is Blue. Lower homes are Blue Right and Red Left. */
    List<Translation2d> LOWER_HOMES = List.of(new Translation2d(2, 1.75), new Translation2d(14.5, 1.75));

    /** Offset is from the "forward" direction. Should be set to Math.PI (180 deg) for a backwards-facing shooter. */
    double SHOOTER_ROTATION_OFFSET = Math.PI;

    /** Lookup table mapping distance from the Hub to the ideal static flywheel speed. */
    // @formatter:off
    InterpolatingDoubleTreeMap HUB_FLYWHEEL_SPEEDS_MAP =
    InterpolatingDoubleTreeMap.ofEntries(
        // Map.entry(Distance in Meters, Flywheel Speed in m/s)
        Map.entry(2.28, 14.35),
        Map.entry(3.09, 14.85),
        Map.entry(4.14, 16.39),
        Map.entry(4.9, 17.55));
    /** Lookup table mapping distance from the current Alliance Home to the ideal static flywheel speed. */
    InterpolatingDoubleTreeMap HOME_FLYWHEEL_SPEEDS_MAP =
    InterpolatingDoubleTreeMap.ofEntries(
        // Map.entry(Distance in Meters, Flywheel Speed in m/s)
        Map.entry(2.28, 14.35),
        Map.entry(3.09, 14.85),
        Map.entry(4.14, 16.39),
        Map.entry(4.9, 17.55)); // @formatter:on
}
