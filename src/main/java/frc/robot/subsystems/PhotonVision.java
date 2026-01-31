package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    /** Standard deviations to weight vision pose updates. (Higher values weight vision less.) */
    private static final Matrix<N3, N1> stddev = VecBuilder.fill(1, 1, 1);

    public static final PhotonCamera CAMERA = new PhotonCamera("C922_Pro_Stream_Webcam");
    /** The Pose offset of the robot from the camera's position */
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(-0.114, 0, .1, new Rotation3d(0, 0, 0));
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); // change depending on field specs

    private static final PhotonPoseEstimator ESTIMATOR = new PhotonPoseEstimator(FIELD_LAYOUT, ROBOT_TO_CAMERA);
    private final EstimateConsumer estConsumer;

    public PhotonVision(EstimateConsumer estimateConsumer) {
        this.estConsumer = estimateConsumer;
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult change : CAMERA.getAllUnreadResults()) {
            visionEst = ESTIMATOR.estimateCoprocMultiTagPose(change);
            if (visionEst.isEmpty()) {
                visionEst = ESTIMATOR.estimateLowestAmbiguityPose(change);
            }
            visionEst.ifPresent(est -> {
                estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, stddev);
            });
        }
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
