package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RotateToHub;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Utilities.*;

/**
 * Handles all telemetry and Elastic/SmartDashboard interactions.
 */
public class DashboardManager {

    // =====================================
    // Simulation
    // =====================================
    public static void setupSimulation(Field2d debugField) {
        SmartDashboard.putData("Simulated Debug Field", debugField);
    }

    // =====================================
    // Robot Overview (Init & Periodic)
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

    public static void updateRobotPeriodic(Drivetrain drivetrain) {
        SmartDashboard.putBoolean("Within Shooting Angle", withinShootingAngle(drivetrain));
        SmartDashboard.putBoolean("Within Shooting Distance", withinShootingDistance(drivetrain));
        SmartDashboard.putBoolean("In Home", inHome(drivetrain));
    }

    // =====================================
    // Subsystem: Hopper
    // =====================================
    public static void setupHopper() {
        SmartDashboard.putNumber("Kicker Test Voltage", 8);
    }

    public static double getKickerTestVoltage() {
        return SmartDashboard.getNumber("Kicker Test Voltage", 0.0);
    }

    // =====================================
    // Subsystem: Flywheel
    // =====================================
    public static void setupFlywheel(DoubleSupplier currentSpeedSupplier, DoubleSupplier targetSpeedSupplier) {
        SmartDashboard.putNumber("Flywheel Test Voltage", 5);
        SmartDashboard.putData("Flywheel Telemetry", builder -> {
            builder.addDoubleProperty("Current Speed (m∕s)", () -> round(currentSpeedSupplier.getAsDouble(), 3), null);
            builder.addDoubleProperty("Target Speed (m∕s)", () -> round(targetSpeedSupplier.getAsDouble(), 3), null);
        });
    }

    public static double getFlywheelTestVoltage() {
        return SmartDashboard.getNumber("Flywheel Test Voltage", 0.0);
    }

    // =====================================
    // Controllers
    // =====================================
    public static void setupController(SendableChooser<Controller> controllerChooser) {
        SmartDashboard.putData("Controller Chooser", controllerChooser);
    }

    // =====================================
    // Commands: RotateToHub
    // =====================================
    public static void updateRotateToHubInit(boolean useShootingCalculator) {
        SmartDashboard.putBoolean("Enable Shooting Calculator", useShootingCalculator);
    }

    public static void updateRotateToHub(Pose2d currentPose, double targetHeading) {
        SmartDashboard.putNumber("Robot Rotation", round(currentPose.getRotation().getDegrees(), 2));
        SmartDashboard.putNumber("Target Heading", round(Math.toDegrees(targetHeading), 2));
        SmartDashboard.putNumber("Angle Difference",
            round(Math.toDegrees(targetHeading - currentPose.getRotation().getRadians()), 2));
    }
}
