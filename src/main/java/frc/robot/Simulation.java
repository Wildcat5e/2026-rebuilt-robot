package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class Simulation {
    private final VisionSystemSim visionSim = new VisionSystemSim("Main");
    private final Drivetrain      drivetrain;

    public Simulation(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        visionSim.addAprilTags(Constants.FIELD_LAYOUT);
        visionSim.addCamera(photonCamera(), PhotonVision.ROBOT_TO_CAMERA);
        DashboardManager.setupSimulation(visionSim.getDebugField());
    }

    private static PhotonCameraSim photonCamera() {
        var cameraSim = new PhotonCameraSim(
                PhotonVision.CAMERA,
                new SimCameraProperties()
                        .setCalibration(1280, 720, Rotation2d.fromDegrees(100))
                        .setCalibError(0.15, 0.05) // approximate detection noise
                        .setFPS(30) // (Note: this is limited by robot loop rate).
                        .setAvgLatencyMs(35) // image data latency
                        .setLatencyStdDevMs(5));
        // Enable drawing a wireframe visualization of the field to the camera streams. Extremely resource-intensive!
        cameraSim.enableDrawWireframe(true);
        return cameraSim;
    }

    public void poseUpdate() {
        visionSim.update(drivetrain.getState().Pose);
    }
}
