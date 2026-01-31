package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    // Standard deviations to weight vision pose updates. (Higher values weight vision less.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);

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
        for (PhotonPipelineResult result : CAMERA.getAllUnreadResults()) {
            visionEst = ESTIMATOR.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = ESTIMATOR.estimateLowestAmbiguityPose(result);
            }
            final Matrix<N3, N1> stddev = getEstimationStdDevs(visionEst, result.getTargets());
            visionEst.ifPresent(est -> {
                estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, stddev);
            });
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags. Credit:
     * https://github.com/PhotonVision/photonvision/blob/v2026.1.1/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     * @return Standard deviation
     */
    private Matrix<N3, N1> getEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
        List<PhotonTrackedTarget> targets) {
        Matrix<N3, N1> estStdDevs = SINGLE_TAG_STD_DEV;
        if (estimatedPose.isEmpty()) {
            return estStdDevs;
        }
        // Pose present. Start running Heuristic
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            var tagPose = ESTIMATOR.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) { // No tags visible.
            return estStdDevs;
        }
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = MULTI_TAG_STD_DEV;
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return estStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
