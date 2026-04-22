package frc.robot.utilities;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * import static frc.robot.utilities.DrivetrainUtils.*;
 * 
 * Utility interface containing custom, reusable commands for the swerve drivetrain.
 */
public interface DrivetrainUtils {
    static final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    /**
     * Creates a command that applies the SwerveDriveBrake request to point the wheels inward in an "X" stance,
     * resisting being pushed.
     * 
     * @param drivetrain The robot's auto-generated Drivetrain instance.
     */
    static Command swerveDriveBrake(Drivetrain drivetrain) {
        return drivetrain.applyRequest(() -> brakeRequest);
    }
}
