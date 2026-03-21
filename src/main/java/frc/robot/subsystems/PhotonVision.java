package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
    // Standard deviations to weight vision pose updates. (Higher values weight vision less.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);

    public static final List<PhotonCamera> CAMERAS =
        List.of(new PhotonCamera("C922_Pro_Stream_Webcam"), new PhotonCamera("HD_Pro_Webcam_C920"));
    /** Offset from center of the robot to camera mount position (robot ➔ camera) in the Robot Coordinate System. */
    public static final List<Transform3d> ROBOT_TO_CAMERA =
        List.of(new Transform3d(-0.07, .295, .57, new Rotation3d(0, 0, 0)),
            new Transform3d(-0.07, .295, .57, new Rotation3d(0, 0, 0)));
    private static final List<PhotonPoseEstimator> ESTIMATORS = new ArrayList<PhotonPoseEstimator>();
    private final EstimateConsumer estConsumer;

    public PhotonVision(EstimateConsumer estimateConsumer) {
        this.estConsumer = estimateConsumer;
        for (int i = 0; i < CAMERAS.size(); i++) {
            ESTIMATORS.add(new PhotonPoseEstimator(Constants.FIELD_LAYOUT, ROBOT_TO_CAMERA.get(i)));
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < CAMERAS.size(); i++) {
            processResults(i);
        }
    }

    private void processResults(int index) {
        for (PhotonPipelineResult result : CAMERAS.get(index).getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = ESTIMATORS.get(index).estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = ESTIMATORS.get(index).estimateLowestAmbiguityPose(result);
            }
            final Matrix<N3, N1> stddev = getEstimationStdDevs(visionEst, result.getTargets(), index);
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
     * @param index Index of current camera and estimator
     * @return Standard deviation
     */
    private Matrix<N3, N1> getEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
        List<PhotonTrackedTarget> targets, int index) {
        Matrix<N3, N1> estStdDevs = SINGLE_TAG_STD_DEV;
        if (estimatedPose.isEmpty()) return estStdDevs;

        // Pose present. Start running Heuristic
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            var tagPose = ESTIMATORS.get(index).getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }
        // No tags visible.
        if (numTags == 0) return estStdDevs;
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = MULTI_TAG_STD_DEV;
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
