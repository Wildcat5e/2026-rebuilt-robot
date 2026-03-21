package frc.robot;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;

public class Simulation {
    private final VisionSystemSim visionSim = new VisionSystemSim("Main");
    /** Enable drawing a wireframe visualization of the field to the camera streams. Extremely resource-intensive! */
    private final boolean ENABLE_WIREFRAME = true;
    private final Drivetrain drivetrain;

    public Simulation(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        visionSim.addAprilTags(Constants.FIELD_LAYOUT);
        for (int i = 0; i < PhotonVision.CAMERAS.size(); i++) {
            final var cameraSim = new PhotonCameraSim(PhotonVision.CAMERAS.get(i), cameraSettings());
            visionSim.addCamera(cameraSim, PhotonVision.ROBOT_TO_CAMERA.get(i));
            cameraSim.enableDrawWireframe(ENABLE_WIREFRAME);
        }
        DashboardManager.setupSimulation(visionSim.getDebugField());
    }

    public void poseUpdate() {
        visionSim.update(drivetrain.getState().Pose);
    }

    private SimCameraProperties cameraSettings() {
        final SimCameraProperties settings = new SimCameraProperties();
        settings.setCalibration(1280, 720, Rotation2d.fromDegrees(100));
        settings.setCalibError(0.15, 0.05); // approximate detection noise
        settings.setFPS(30); // (Note: this is limited by robot loop rate).
        settings.setAvgLatencyMs(35); // image data latency
        settings.setLatencyStdDevMs(5); // image data latency
        return settings;
    }
}
