package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DashboardManager;

public class PhotonVision extends SubsystemBase {
    // Standard deviations to weight vision pose updates. (Higher values weight vision less.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);

    public static final List<PhotonCamera> CAMERAS = List.of(new PhotonCamera("Microsoft_LifeCam_HD-3000-Left-USB2-Vert"),
        new PhotonCamera("Microsoft_LifeCam_HD-3000-Right"));
    /** Offset from center of the robot to camera mount position (robot ➔ camera) in the Robot Coordinate System. */
    public static final List<Transform3d> ROBOT_TO_CAMERA =
        List.of(new Transform3d(0.305, 0.221, 0.521, new Rotation3d(Math.toRadians(3), Math.toRadians(-1), 0)),
            new Transform3d(0.69, -0.221, 0.527, new Rotation3d(0, Math.toRadians(15), 0)));
    private static final List<PhotonPoseEstimator> ESTIMATORS = new ArrayList<PhotonPoseEstimator>();
    private final EstimateConsumer estConsumer;
    public Pose2d leftCamPose = new Pose2d(0, 0, new Rotation2d(0));
    public Pose2d rightCamPose = new Pose2d(0, 0, new Rotation2d(0));

    public PhotonVision(EstimateConsumer estimateConsumer) {
        this.estConsumer = estimateConsumer;
        for (int i = 0; i < CAMERAS.size(); i++) {
            ESTIMATORS.add(new PhotonPoseEstimator(Constants.FIELD_LAYOUT, ROBOT_TO_CAMERA.get(i)));
        }

        DashboardManager.setupCameraToggles();
    }

    @Override
    public void periodic() {
        boolean[] camerasEnabled = {DashboardManager.isLeftCameraEnabled(), DashboardManager.isRightCameraEnabled()};

        for (int i = 0; i < CAMERAS.size(); i++) {
            if (i < camerasEnabled.length && camerasEnabled[i]) {
                processResults(i);
            }
        }
    }

    private void processResults(int index) {
        for (PhotonPipelineResult result : CAMERAS.get(index).getAllUnreadResults()) {
            var optionalPoseEstimate = ESTIMATORS.get(index).estimateCoprocMultiTagPose(result)
                .or(() -> ESTIMATORS.get(index).estimateLowestAmbiguityPose(result));

            optionalPoseEstimate.ifPresent((est) -> {
                Matrix<N3, N1> stddev = getEstimationStdDevs(est, result, index);
                estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, stddev);
                if (index == 0) {
                    leftCamPose = est.estimatedPose.toPose2d();
                } else {
                    rightCamPose = est.estimatedPose.toPose2d();
                }
            });
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags. Credit:
     * https://github.com/PhotonVision/photonvision/blob/v2026.1.1/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
     *
     * @param estimation The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     * @param index Index of current camera and estimator
     * @return Standard deviation
     */
    private Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimation, PhotonPipelineResult result, int index) {
        Matrix<N3, N1> estStdDevs = SINGLE_TAG_STD_DEV;

        // Pose present. Start running Heuristic
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : result.getTargets()) {
            var tagPose = ESTIMATORS.get(index).getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimation.estimatedPose.toPose2d().getTranslation());
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

    private Matrix<N3, N1> altGetEstStdDevs(PhotonPipelineResult result) {

        // Convert targets to April Tag 2d poses, removing null values
        var tagPoses = result.targets.stream().map(PhotonTrackedTarget::getFiducialId).map(aprilTagPoses::get)
            .filter(Objects::nonNull).toList();

        if (tagPoses.isEmpty()) {
            // No valid April Tag poses for the targets in this result, skip it since we can't calculate distances to tags.
            continue;
        }

        var poseEstimate = optionalPoseEstimate.get();
        var poseEstimate2d = poseEstimate.estimatedPose.toPose2d();

        // Average distance between tag pose and robot pose estimate.
        // This gives us a rough idea of how far the robot is from the tags it sees,
        // which is a major factor in pose estimation accuracy. We use this to adjust our
        // confidence in the pose estimate (i.e. the standard deviation we pass to the consumer).
        var averageDistance =
            tagPoses.stream().map(tagPose -> tagPose.getTranslation().getDistance(poseEstimate2d.getTranslation()))
                .reduce(0.0, Double::sum) / tagPoses.size();

        if (tagPoses.size() == 1 && averageDistance <= 4.0) {
            // Only use single tag pose estimate if the average distance is less than 4 meters.
            // This is an empirical threshold based on testing that balances trusting single tag estimates
            // when they are likely accurate and ignoring them when they are likely inaccurate.
            poseEstimateConsumer.accept(poseEstimate2d, poseEstimate.timestampSeconds, SINGLE_TAG_STD_DEV);
        } else {
            // For multiple tags, we can be more confident in the pose estimate, so we use a lower standard deviation.
            // However, we still want to account for distance to the tags, since farther tags generally lead to less
            // accurate estimates. This heuristic scales the standard deviation based on the average distance to the tags,
            // with a cap at around 4 meters (since beyond that, vision is generally not very reliable).
            var standardDeviation = MULTI_TAG_STD_DEV.times(1 * (averageDistance * averageDistance / 30.0));
            poseEstimateConsumer.accept(poseEstimate2d, poseEstimate.timestampSeconds, standardDeviation);
        }
    }


    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
