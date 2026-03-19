package frc.robot.vision;

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
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Map;
import java.util.Objects;

import static java.util.stream.Collectors.toMap;

public class PhotonVision extends SubsystemBase {
    // Standard deviations to weight vision pose updates. (Higher values weight vision less.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final PoseEstimateConsumer poseEstimateConsumer;
    private final Map<Integer, Pose2d> aprilTagPoses;
    private double numOfTags;
    private double averageDistance;

    /**
     * @param camera The camera to use for pose estimation. Make sure to set the correct camera name and robot-to-camera
     *        transform.
     * @param estimator The photon pose estimator used for pose estimation.
     * @param estimateConsumer The consumer receiving the estimated pose and timestamp.
     */
    public PhotonVision(PhotonCamera camera, PhotonPoseEstimator estimator, PoseEstimateConsumer estimateConsumer) {
        super(camera.getName());
        this.photonCamera = camera;
        this.poseEstimateConsumer = estimateConsumer;
        this.poseEstimator = estimator;
        // Only convert AprilTags to poses once since the field layout is static. Map tag ID to pose for fast lookup during pose estimation.
        // @formatter:off
            this.aprilTagPoses = estimator.getFieldTags()
                                          .getTags()
                                          .stream()
                                          .collect(toMap(tag -> tag.ID, tag -> tag.pose.toPose2d())); // @formatter:on
        DashboardManager.setupVision(() -> numOfTags, () -> averageDistance);
    }


    public PhotonVision(PoseEstimateConsumer estimateConsumer) {
        this(new PhotonCamera("HD_Pro_Webcam_C920"),
            new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
                new Transform3d(-0.07, .295, .57, Rotation3d.kZero)),
            estimateConsumer);
    }

    @Override
    public void periodic() {
        for (PhotonPipelineResult result : photonCamera.getAllUnreadResults()) {

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

            numOfTags = tagPoses.size();

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
            averageDistance =
                tagPoses.stream().map(tagPose -> tagPose.getTranslation().getDistance(poseEstimate2d.getTranslation()))
                    .reduce(0.0, Double::sum) / numOfTags;


            if (numOfTags > 1) {
                // For multiple tags, we can be more confident in the pose estimate, so we use a lower standard deviation.
                // However, we still want to account for distance to the tags, since farther tags generally lead to less
                // accurate estimates. This heuristic scales the standard deviation based on the average distance to the tags,
                // with a cap at around 4 meters (since beyond that, vision is generally not very reliable).
                var standardDeviation = MULTI_TAG_STD_DEV.times(1 * (averageDistance * averageDistance / 30.0));
                poseEstimateConsumer.accept(poseEstimate2d, poseEstimate.timestampSeconds, standardDeviation);
            } else if (averageDistance <= 4.0) {
                // Only use single tag pose estimate if the average distance is less than 4 meters.
                // This is an empirical threshold based on testing that balances trusting single tag estimates
                // when they are likely accurate and ignoring them when they are likely inaccurate.
                poseEstimateConsumer.accept(poseEstimate2d, poseEstimate.timestampSeconds, SINGLE_TAG_STD_DEV);
            }
        }
    }
}
