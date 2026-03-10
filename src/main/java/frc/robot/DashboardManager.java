package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RotateToHub;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Utilities.*;

/** Handles all telemetry and Elastic/SmartDashboard interactions. */
public interface DashboardManager {

    // =====================================
    // Simulation
    // =====================================
    static void setupSimulation(Field2d debugField) {
        SmartDashboard.putData("Simulated Debug Field", debugField);
    }

    // =====================================
    // Robot (Init & Periodic)
    // =====================================
    public static void setupRobotInit(Field2d fieldWidget, SendableChooser<Command> autoChooser,
        Drivetrain drivetrain) {
        SmartDashboard.putData("Field", fieldWidget);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Auto Command Chooser", autoChooser);
        SmartDashboard.putData("RotateToHub PID Controller", RotateToHub.PID_CONTROLLER);
        SmartDashboard.putData("Robot Telemetry", builder -> {
            builder.addDoubleProperty("Distance to Hub (m)", () -> round(getHubDistance(drivetrain), 3), null);
        });
    }

    static void updateRobotPeriodic(Drivetrain drivetrain) {
        SmartDashboard.putBoolean("Within Shooting Angle", withinShootingAngle(drivetrain));
        SmartDashboard.putBoolean("Within Shooting Distance", withinShootingDistance(drivetrain));
        SmartDashboard.putBoolean("In Home", inHome(drivetrain));
    }

    // =====================================
    // Subsystem: Hopper
    // =====================================
    static void setupHopper() {
        SmartDashboard.putNumber("Kicker Test Voltage", 8);
    }

    static double getKickerTestVoltage() {
        return SmartDashboard.getNumber("Kicker Test Voltage", 0.0);
    }

    // =====================================
    // Subsystem: Flywheel
    // =====================================
    static void setupFlywheel(DoubleSupplier currentSpeedSupplier, DoubleSupplier targetSpeedSupplier,
        DoubleSupplier averageSpeedSupplier) {
        SmartDashboard.putNumber("Flywheel Test Voltage", 5);
        SmartDashboard.putNumber("Calculated Voltage", 0);
        SmartDashboard.putNumber("Current Speed (m\u2215s)", 0);
        SmartDashboard.putNumber("Target Speed (m\u2215s)", 0); // unicode needs to be like this to not break
        SmartDashboard.putData("Flywheel Telemetry", builder -> {
            builder.addDoubleProperty("Current Speed (m\u2215s)", () -> round(currentSpeedSupplier.getAsDouble(), 3),
                null);
            builder.addDoubleProperty("5s Avg Speed (m\u2215s)", () -> round(averageSpeedSupplier.getAsDouble(), 3),
                null);
            builder.addDoubleProperty("Target Speed (m\u2215s)", () -> round(targetSpeedSupplier.getAsDouble(), 3),
                null);
        });
    }

    static double getFlywheelTestVoltage() {
        return SmartDashboard.getNumber("Flywheel Test Voltage", 0.0);
    }

    // =====================================
    // Controllers
    // =====================================
    static void setupController(SendableChooser<Controller> controllerChooser) {
        SmartDashboard.putData("Controller Chooser", controllerChooser);
    }

    // =====================================
    // Commands: RotateToHub
    // =====================================
    static void updateRotateToHubInit(boolean useShootingCalculator) {
        SmartDashboard.putBoolean("Enable Shooting Calculator", useShootingCalculator);
    }

    static void updateRotateToHub(Pose2d currentPose, double targetHeading) {
        SmartDashboard.putNumber("Robot Rotation", round(currentPose.getRotation().getDegrees(), 2));
        SmartDashboard.putNumber("Target Heading", round(Math.toDegrees(targetHeading), 2));
        SmartDashboard.putNumber("Angle Difference",
            round(Math.toDegrees(targetHeading - currentPose.getRotation().getRadians()), 2));
    }


    // =====================================

    // Debugging

    // =====================================
    // Use these methods to quickly post debug values to Elastic.
    // By prefixing the keys with "Debug/", NetworkTables and Elastic will
    // automatically group them into a collapsible "Debug" folder.

    /** Posts a debug number to the SmartDashboard under the "Debug/" folder. */
    static void putDebugNumber(String key, double value) {
        SmartDashboard.putNumber("Debug/" + key, value);
    }

    /** Posts a debug boolean to the SmartDashboard under the "Debug/" folder. */
    static void putDebugBoolean(String key, boolean value) {
        SmartDashboard.putBoolean("Debug/" + key, value);
    }

    /** Posts a debug string to the SmartDashboard under the "Debug/" folder. */
    static void putDebugString(String key, String value) {
        SmartDashboard.putString("Debug/" + key, value);
    }

    /** Posts a debug Sendable object to the SmartDashboard under the "Debug/" folder. */
    static void putDebugData(String key, Sendable data) {
        SmartDashboard.putData("Debug/" + key, data);
    }
}
