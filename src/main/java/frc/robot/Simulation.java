package frc.robot;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;

public class Simulation {
    private final VisionSystemSim visionSim = new VisionSystemSim("Main");
    /** Enable drawing a wireframe visualization of the field to the camera streams. Extremely resource-intensive! */
    private final boolean ENABLE_WIREFRAME = true;
    private final Drivetrain drivetrain;
    private boolean defaultAllianceNotSet = true;

    public Simulation(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        visionSim.addAprilTags(Constants.FIELD_LAYOUT);
        final PhotonCameraSim cameraSim = new PhotonCameraSim(PhotonVision.CAMERA, cameraSettings());
        visionSim.addCamera(cameraSim, PhotonVision.ROBOT_TO_CAMERA);
        cameraSim.enableDrawWireframe(ENABLE_WIREFRAME);
        SmartDashboard.putData("Simulated Debug Field", visionSim.getDebugField());
    }

    public void poseUpdate() {
        visionSim.update(drivetrain.getState().Pose);
    }

    public void checkAndSetDefaultAlliance() {
        if (defaultAllianceNotSet) {
            // Set (default) alliance color on sim start. 
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            var fmsTable = NetworkTableInstance.getDefault().getTable("FMSInfo");
            fmsTable.getEntry("IsRedAlliance").setBoolean(false);
            fmsTable.getEntry("StationNumber").setDouble(1);
            System.out.println("attempting set blue");
            if (!fmsTable.getEntry("IsRedAlliance").getBoolean(true)
                && DriverStationSim.getAllianceStationId() == AllianceStationID.Blue1) {
                System.out.println("success");
                //defaultAllianceNotSet = false;
            } else System.out.println("failed");
        }
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
