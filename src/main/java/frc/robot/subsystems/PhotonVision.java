package frc.robot.subsystems;


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
import frc.robot.DashboardManager;
import frc.robot.vision.PoseEstimateConsumer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Map;
import java.util.Objects;

import static java.util.stream.Collectors.toMap;
import java.util.List;

public class PhotonVision extends SubsystemBase {
    // Standard deviations to weight vision pose updates. (Higher values weight vision less.)
    public static final Matrix<N3, N1> MAX_VALUE_STD_DEV =
        VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE);
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);

    public static PhotonCamera photonCamera1 = new PhotonCamera("HD_Pro_Webcam_C920");
    public static PhotonCamera photonCamera2 = new PhotonCamera("C922_Pro_Stream_Webcam");
    public static Transform3d ROBOT_TO_CAMERA1 = new Transform3d(-0.310, 0.213, 0.292, new Rotation3d(1.501, 0, 0));
    public static Transform3d ROBOT_TO_CAMERA2 =
        new Transform3d(-0.315, -0.217, 0.292, new Rotation3d(4.712, 0.087, 0));
    private PhotonPoseEstimator poseEstimator1 =
        new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), ROBOT_TO_CAMERA1);
    private PhotonPoseEstimator poseEstimator2 =
        new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), ROBOT_TO_CAMERA2);
    private final PoseEstimateConsumer poseEstimateConsumer;
    private final Map<Integer, Pose2d> aprilTagPoses;
    private double numOfTagsCam1 = 0;
    private double averageDistanceCam1 = 0;
    private String stddevCategoryCam1 = "DEFAULT";
    private double stddevXCam1 = 0;
    private double stddevYCam1 = 0;
    private double stddevRotationCam1 = 0;
    private double numOfTagsCam2 = 0;
    private double averageDistanceCam2 = 0;
    private String stddevCategoryCam2 = "DEFAULT";
    private double stddevXCam2 = 0;
    private double stddevYCam2 = 0;
    private double stddevRotationCam2 = 0;

    /**
     * @param camera The camera to use for pose estimation. Make sure to set the correct camera name and robot-to-camera
     *        transform.
     * @param estimator The photon pose estimator used for pose estimation.
     * @param estimateConsumer The consumer receiving the estimated pose and timestamp.
     */
    public PhotonVision(PoseEstimateConsumer estimateConsumer) {
        super("PhotonVision");
        this.poseEstimateConsumer = estimateConsumer;
        // Only convert AprilTags to poses once since the field layout is static. Map tag ID to pose for fast lookup during pose estimation.
        // @formatter:off
            this.aprilTagPoses = poseEstimator1.getFieldTags()
                                          .getTags()
                                          .stream()
                                          .collect(toMap(tag -> tag.ID, tag -> tag.pose.toPose2d())); // @formatter:on
        DashboardManager.setupVision(() -> numOfTagsCam1, () -> averageDistanceCam1, () -> stddevXCam1,
            () -> stddevYCam1, () -> stddevRotationCam1, () -> stddevCategoryCam1, () -> numOfTagsCam2,
            () -> averageDistanceCam2, () -> stddevXCam2, () -> stddevYCam2, () -> stddevRotationCam2,
            () -> stddevCategoryCam2);
    }


    @Override
    public void periodic() {
        updateVisionPoseFromCamera(photonCamera1, poseEstimator1);
        updateVisionPoseFromCamera(photonCamera2, poseEstimator2);
    }

    private void updateVisionPoseFromCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

            // Use coprocessor if available, otherwise use lowest ambiguity pose.
            var optionalPoseEstimate = poseEstimator.estimateCoprocMultiTagPose(result)
                .or(() -> poseEstimator.estimateLowestAmbiguityPose(result));

            if (optionalPoseEstimate.isEmpty()) {
                // No valid pose estimate for this result, skip it.
                continue;
            }

            // Convert targets to April Tag 2d poses, removing null values
            // @formatter:off
                var tagPoses = result.targets.stream()
                                             .map(PhotonTrackedTarget::getFiducialId)
                                             .map(aprilTagPoses::get)
                                             .filter(Objects::nonNull)
                                             .toList(); // @formatter:on


            if (tagPoses.isEmpty()) {
                // No valid April Tag poses for the targets in this result, skip it since we can't calculate distances to tags.
                continue;
            }

            var poseEstimate = optionalPoseEstimate.get();
            var poseEstimate2d = poseEstimate.estimatedPose.toPose2d();

            var standardDeviation = getStandardDeviation(tagPoses, poseEstimate2d);
            if (camera == photonCamera1) {
                averageDistanceCam1 = tagPoses.stream()
                    .map(tagPose -> tagPose.getTranslation().getDistance(poseEstimate2d.getTranslation()))
                    .reduce(0.0, Double::sum) / tagPoses.size();
                stddevXCam1 = standardDeviation.get(0, 0);
                stddevYCam1 = standardDeviation.get(1, 0);
                stddevRotationCam1 = standardDeviation.get(2, 0);
                numOfTagsCam1 = tagPoses.size();
            } else {
                averageDistanceCam2 = tagPoses.stream()
                    .map(tagPose -> tagPose.getTranslation().getDistance(poseEstimate2d.getTranslation()))
                    .reduce(0.0, Double::sum) / tagPoses.size();
                stddevXCam2 = standardDeviation.get(0, 0);
                stddevYCam2 = standardDeviation.get(1, 0);
                stddevRotationCam2 = standardDeviation.get(2, 0);
                numOfTagsCam2 = tagPoses.size();;
            }
            poseEstimateConsumer.accept(poseEstimate2d, poseEstimate.timestampSeconds, standardDeviation);
        }
    }

    private Matrix<N3, N1> getStandardDeviation(List<Pose2d> tagPoses, Pose2d poseEstimate2d) {
        var numOfTags = tagPoses.size();
        // Average distance between tag pose and robot pose estimate.
        // This gives us a rough idea of how far the robot is from the tags it sees,
        // which is a major factor in pose estimation accuracy. We use this to adjust our
        // confidence in the pose estimate (i.e. the standard deviation we pass to the consumer).
        var averageDistance =
            tagPoses.stream().map(tagPose -> tagPose.getTranslation().getDistance(poseEstimate2d.getTranslation()))
                .reduce(0.0, Double::sum) / numOfTags;

        if (numOfTags > 1) {
            // For multiple tags, we can be more confident in the pose estimate, so we use a lower standard deviation.
            // However, we still want to account for distance to the tags, since farther tags generally lead to less
            // accurate estimates. This heuristic scales the standard deviation based on the average distance to the tags,
            // with a cap at around 4 meters (since beyond that, vision is generally not very reliable).
            Matrix<N3, N1> standardDeviation = MULTI_TAG_STD_DEV.times(
                DashboardManager.getStandardDeviationMultiplier() * (1 * (averageDistance * averageDistance / 30.0)));

            return standardDeviation;
        } else if (averageDistance <= 4.0) {
            // Only use single tag pose estimate if the average distance is less than 4 meters.
            // This is an empirical threshold based on testing that balances trusting single tag estimates
            // when they are likely accurate and ignoring them when they are likely inaccurate.
            Matrix<N3, N1> standardDeviation =
                SINGLE_TAG_STD_DEV.times(DashboardManager.getStandardDeviationMultiplier());
            return standardDeviation;
        } else {
            return MAX_VALUE_STD_DEV;
        }
    }
}

