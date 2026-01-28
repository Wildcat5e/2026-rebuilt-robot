package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import static frc.robot.Utilities.*;

public class PhotonVision extends SubsystemBase {
    /** Standard deviations to weight vision pose updates. (Higher values weight vision less.) */
    private static final Matrix<N3, N1> stddev = VecBuilder.fill(1, 1, 1);

    public static final PhotonCamera CAMERA = new PhotonCamera("C922_Pro_Stream_Webcam");
    /** The Pose offset of the robot from the camera's position */
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(-0.114, 0, .1, new Rotation3d(0, 0, 0));
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); // change depending on field specs

    private static final PhotonPoseEstimator ESTIMATOR =
        new PhotonPoseEstimator(FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAMERA); // Deprecated without any replacement so we keep using it.
    private final Drivetrain drivetrain;
    DoublePublisher distancePublisher;

    public PhotonVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;


    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Within Shooting Angle", withinShootingAngle());
        SmartDashboard.putBoolean("Within Shooting Distance", withinShootingDistance());
        // get all vision updates and loop through them
        for (PhotonPipelineResult change : CAMERA.getAllUnreadResults()) {
            // takes camera image and passes into photon estimator, returning pose data object
            Optional<EstimatedRobotPose> optionalVisionEst = ESTIMATOR.update(change);
            if (optionalVisionEst.isEmpty()) {
                continue;
            }
            EstimatedRobotPose visionEst = optionalVisionEst.get();
            Pose2d estimatedPose2d = visionEst.estimatedPose.toPose2d(); // turns the estimate into a pose
            drivetrain.addVisionMeasurement(estimatedPose2d, visionEst.timestampSeconds, stddev);
        }
    }


}
