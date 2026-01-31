package frc.robot.subsystems;

/**
 * A helper class to calculate shooting parameters while moving.<br>
 * <br>
 * Assumptions: <br>
 * 1) Robot position and velocity are Field-Centric.<br>
 * 2) Hood angle is fixed.<br>
 * 3) A lookup table exists for static shooting.<br>
 */
public class ShootingCalculator {

    public static class ShotSolution {
        /**
         * The flywheel speed can be in RPM or m/s depending on how the flywheel controller is set up. It all comes down
         * to what units the lookup table provides.
         */
        public double flywheelSpeed;
        public double robotHeading; // The field-centric angle (degrees) the robot should face
        public double distanceToTarget;

        public ShotSolution(double speed, double heading, double dist) {
            this.flywheelSpeed = speed;
            this.robotHeading = heading;
            this.distanceToTarget = dist;
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
    private final double FIXED_HOOD_ANGLE_RADIANS;

    /**
     * @param fixedHoodAngleDegrees The constant angle of the shooter in degrees.
     */
    public ShootingCalculator(double fixedHoodAngleDegrees) {
        this.FIXED_HOOD_ANGLE_RADIANS = Math.toRadians(fixedHoodAngleDegrees);
    }

    /**
     * Calculates the necessary robot heading and shot speed to hit the target while moving.
     *
     * @param robotPos Field-centric robot position (x, y)
     * @param robotVel Field-centric robot velocity (vx, vy)
     * @param targetPos Field-centric target position (x, y) - Z is handled by lookup table logic
     * @return ShotSolution containing new heading and speed
     */
    public ShotSolution calculate(Vector2d robotPos, Vector2d robotVel, Vector2d targetPos) {

        // 1. Calculate Vector to Target (Distance and Angle)
        double dx = targetPos.x - robotPos.x;
        double dy = targetPos.y - robotPos.y;
        double distanceToTarget = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dy, dx);

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
        double shotVx = staticVx - robotVel.x;
        double shotVy = staticVy - robotVel.y;

        // 6. Extract Outputs

        // Calculate the new Heading (Lead Angle)
        double newHeadingRadians = Math.atan2(shotVy, shotVx);

        // Calculate the new Shot Speed
        // First get the horizontal magnitude
        double newShotHorizontalSpeed = Math.hypot(shotVx, shotVy);

        // Then convert back to full flywheel speed (divide by cos(hood))
        // S_total = V_horizontal / cos(theta)
        double newFlywheelSpeed = newShotHorizontalSpeed / Math.cos(FIXED_HOOD_ANGLE_RADIANS);

        return new ShotSolution(newFlywheelSpeed, newHeadingRadians, distanceToTarget);
    }

    /**
     * Mock Lookup Table
     */
    private double getStaticSpeedFromTable(double distance) {
        // Probably going to use InterpolatingDoubleTreeMap or similar.

        // Dummy formula: Speed increases with distance
        return 10.0 + (distance * 2.0);
    }
}
