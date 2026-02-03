package frc.robot.subsystems;

import static frc.robot.Utilities.getHubDistance;
import static frc.robot.Utilities.getRobotToHubAngle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


/**
 * A helper class to calculate shooting parameters while moving.<br>
 * <br>
 * Assumptions: <br>
 * 1) Robot position and velocity are Field-Centric.<br>
 * 2) Hood angle is fixed.<br>
 * 3) A lookup table exists for static shooting.<br>
 */
public class ShootingCalculator {
    // PLACE HOLDER VALUE !!
    static final double FIXED_HOOD_ANGLE_RADIANS = 10;

    public static class ShotSolution {
        /**
         * The flywheel speed will be in m/s, but can be converted to RPM using the formula:<br>
         * <br>
         * RPM = (flywheelSpeed * 60) / (2 * π * radius)
         */
        public double flywheelSpeed;
        public double robotHeading; // The field-centric angle (radians) the robot should face
        public double distanceToTarget;

        public ShotSolution(double speed, double heading) {
            this.flywheelSpeed = speed;
            this.robotHeading = heading;
        }
    }

    // Simple 2D point/vector
    public static class Vector2d {
        public double x;
        public double y;

        public Vector2d(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    // Constant

    /**
     * Constructs a ShootingCalculator with a fixed hood angle in degrees.
     *
     * This constructor defaults to assuming the input is in degrees.<br>
     * 
     * @param fixedHoodAngleDegrees The constant angle of the shooter in degrees.
     */
    // public ShootingCalculator(double hoodAngleDegrees) {
    //     // Calls the main constructor with 'false' for inputInRadians
    //     this(hoodAngleDegrees, false);
    // }

    /**
     * Constructs a ShootingCalculator with a fixed hood angle, specifying the unit.<br>
     * 
     * @param angle The constant angle of the shooter.
     * 
     * @param inputInRadians Set to true if the input angle is in Radians; set to false if the input angle is in
     *        Degrees.
     */
    // public ShootingCalculator(double hoodAngle, boolean inputInRadians) {
    //     this.FIXED_HOOD_ANGLE_RADIANS = inputInRadians ? hoodAngle : Math.toRadians(hoodAngle);
    // }

    /**
     * Calculates the necessary robot heading and shot speed to hit the target while moving.
     *
     * @param robotVel Robot-centric robot velocity (vx, vy) (use "drivetrain.getState().Speeds" for robot-centric
     *        speed)
     * @return ShotSolution containing new heading and speed
     */
    static public ShotSolution calculate() {
        ChassisSpeeds robotVel = Controller.drivetrain.getState().Speeds;
        // Convert robot centric speeds to field centric speeds
        robotVel =
            ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, Controller.drivetrain.getState().Pose.getRotation());
        double robotVelX = robotVel.vxMetersPerSecond;
        double robotVelY = robotVel.vyMetersPerSecond;

        // 1. Calculate Vector to Target (Distance and Angle)
        double distanceToTarget = getHubDistance();
        double angleToTarget = getRobotToHubAngle();

        // 2. Look up the Ideal "Static" Shot Speed This is the speed you would shoot if standing perfectly still at
        // this distance.
        double staticSpeed = getStaticSpeedFromTable(distanceToTarget);

        // 3. Decompose Static Shot into Horizontal Component
        // We only care about the horizontal plane for vector subtraction.
        // horizontal_speed = total_speed * cos(hood_angle)
        double staticSpeedHorizontal = staticSpeed * Math.cos(FIXED_HOOD_ANGLE_RADIANS);

        // 4. Create the Static Vector
        // This vector points directly at the hub with the magnitude calculated above.
        double staticVx = staticSpeedHorizontal * Math.cos(angleToTarget);
        double staticVy = staticSpeedHorizontal * Math.sin(angleToTarget);

        // 5. Calculate the Shot Vector (Vector Subtraction)
        // V_shot = V_static - V_robot
        double shotVx = staticVx - robotVelX;
        double shotVy = staticVy - robotVelY;

        // 6. Extract Outputs

        // Calculate the new Heading (Lead Angle)
        double newHeadingRadians = Math.atan2(shotVy, shotVx);

        // Calculate the new Shot Speed
        // First get the horizontal magnitude
        double newShotHorizontalSpeed = Math.hypot(shotVx, shotVy);

        // Then convert back to full flywheel speed (divide by cos(hood))
        // S_total = V_horizontal / cos(theta)
        double newFlywheelSpeed = newShotHorizontalSpeed / Math.cos(FIXED_HOOD_ANGLE_RADIANS);

        return new ShotSolution(newFlywheelSpeed, newHeadingRadians);
    }

    /**
     * Mock Lookup Table
     */
    static private double getStaticSpeedFromTable(double distance) {
        // Probably going to use InterpolatingDoubleTreeMap or similar.

        // Dummy formula: Speed increases with distance
        return 10.0 + (distance * 2.0);
    }
}
