package frc.robot.utilities;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * A utility class containing custom, reusable commands for the swerve drivetrain. This keeps custom logic separated
 * from the auto-generated Drivetrain.java file, so nothing is overwritten by Phoenix Tuner X.
 */
public class DrivetrainUtils {
    private static final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    /**
     * Creates a command that applies the SwerveDriveBrake request to point the wheels inward in an "X" stance,
     * resisting being pushed.
     * 
     * @param drivetrain The robot's auto-generated Drivetrain instance.
     */
    public static Command swerveDriveBrake(Drivetrain drivetrain) {
        return drivetrain.applyRequest(() -> brakeRequest);
    }
}
