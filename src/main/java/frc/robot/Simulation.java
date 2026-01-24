package frc.robot;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PhotonVision;

public class Simulation {
    private final VisionSystemSim visionSim = new VisionSystemSim("Main");
    /** Enable drawing a wireframe visualization of the field to the camera streams. Extremely resource-intensive! */
    private final boolean ENABLE_WIREFRAME = true;

    public Simulation() {
        visionSim.addAprilTags(PhotonVision.FIELD_LAYOUT);
        final PhotonCameraSim cameraSim = new PhotonCameraSim(PhotonVision.CAMERA, cameraSettings());
        visionSim.addCamera(cameraSim, PhotonVision.ROBOT_TO_CAMERA);
        cameraSim.enableDrawWireframe(ENABLE_WIREFRAME);
        SmartDashboard.putData("Simulated Debug Field", visionSim.getDebugField());
    }

    public void poseUpdate() {
        visionSim.update(RobotContainer.drivetrain.getState().Pose);
    }

    private SimCameraProperties cameraSettings() {
        final SimCameraProperties settings = new SimCameraProperties();
        settings.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        settings.setCalibError(0.25, 0.08); // approximate detection noise
        settings.setFPS(30); // (Note: this is limited by robot loop rate).
        settings.setAvgLatencyMs(35); // image data latency
        settings.setLatencyStdDevMs(5); // image data latency
        return settings;
    }
}
