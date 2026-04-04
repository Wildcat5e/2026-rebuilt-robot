package frc.robot;

import static frc.robot.utilities.FieldUtils.*;
import static frc.robot.utilities.MatchUtils.*;
import static frc.robot.utilities.MathUtils.*;
import static frc.robot.utilities.TargetingUtils.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AimAtTarget;
import frc.robot.controller.Controller;
import frc.robot.subsystems.Drivetrain;

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
    public static void setupRobotInit(Field2d field, SendableChooser<Command> autoChooser, Drivetrain drivetrain) {
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Auto Command Chooser", autoChooser);
        SmartDashboard.putData("Robot Telemetry", builder -> {
            builder.addDoubleProperty("Battery Voltage (V)", () -> round(RobotController.getBatteryVoltage(), 2), null);
            builder.addDoubleProperty("Distance to Hub (m)",
                () -> round(getTargetDistance(drivetrain, getHubPosition()), 2), null);
        });
        if (!Robot.IS_COMPETITION) SmartDashboard.putData("Aim At Target PID Controller", AimAtTarget.PID_CONTROLLER);
    }

    static void updateRobotPeriodic(Drivetrain drivetrain, Translation2d target) {
        // Remaining time in the current period (Auto/Teleop)
        SmartDashboard.putNumber("Hub Shift and Times/Match Time Remaining", round(DriverStation.getMatchTime(), 1));
        SmartDashboard.putNumber("Hub Shift and Times/Hub Shift Time Remaining", round(getHubShiftTimeRemaining(), 1));
        SmartDashboard.putBoolean("Hub Shift and Times/Is Hub Active", isHubActive());
        SmartDashboard.putString("Hub Shift and Times/Game Specific Message", DriverStation.getGameSpecificMessage());
        SmartDashboard.putBoolean("In Home", inHome(drivetrain));
    }

    // =====================================
    // Subsystem: Hopper
    // =====================================
    static void setupHopper() {
        SmartDashboard.putNumber("Kicker Test Voltage", 8);
        SmartDashboard.putNumber("Conveyor Test Voltage", 8.0);

    }

    static double getKickerTestVoltage() {
        return SmartDashboard.getNumber("Kicker Test Voltage", 0.0);
    }

    static double getConveyorTestVoltage() {
        return SmartDashboard.getNumber("Conveyor Test Voltage", 0.0);
    }

    // =====================================
    // Subsystem: Flywheel
    // =====================================
    static void setupFlywheel(DoubleSupplier currSpeedSupp, DoubleSupplier targetSpeedSupp, DoubleSupplier avgSpeedSupp,
        DoubleSupplier calculatedVoltageSupp) {
        SmartDashboard.putNumber("Flywheel Test Voltage", 5);
        SmartDashboard.putNumber("Flywheel Speed Multiplier", 1.05);
        SmartDashboard.putNumber("Home Flywheel Speed Multiplier", 0.85);
        SmartDashboard.putNumber("Tunable Flywheel Speed", 10);
        SmartDashboard.putData("Flywheel Telemetry", builder -> { // @formatter:off
            builder.addDoubleProperty("Current Flywheel Speed (m\u2215s)", () -> round(currSpeedSupp.getAsDouble(), 1), null);
            builder.addDoubleProperty("5s Avg Flywheel Speed (m\u2215s)", () -> round(avgSpeedSupp.getAsDouble(), 1), null);
            builder.addDoubleProperty("Target Flywheel Speed (m\u2215s)", () -> round(targetSpeedSupp.getAsDouble(), 1), null);
            builder.addDoubleProperty("Calculated Flywheel Voltage", () -> round(calculatedVoltageSupp.getAsDouble(), 1), null);
        }); // @formatter:on
    }

    static double getFlywheelTestVoltage() {
        return SmartDashboard.getNumber("Flywheel Test Voltage", 0.0);
    }

    static double getFlywheelSpeedMultiplier() {
        return SmartDashboard.getNumber("Flywheel Speed Multiplier", 0.0);
    }

    static double getHomeFlywheelSpeedMultiplier() {
        return SmartDashboard.getNumber("Home Flywheel Speed Multiplier", 0.0);
    }

    static double getTunableFlywheelSpeed() {
        return SmartDashboard.getNumber("Tunable Flywheel Speed", 10.0);
    }

    static void incrementFlywheelSpeedMultiplier(double increment) {
        SmartDashboard.putNumber("Flywheel Speed Multiplier", getFlywheelSpeedMultiplier() + increment);
    }

    static void incrementTunableFlywheelSpeed(double increment) {
        SmartDashboard.putNumber("Tunable Flywheel Speed", getTunableFlywheelSpeed() + increment);
    }

    // =====================================
    // Subsystem: Intake 
    // =====================================

    static void setupIntake(DoubleSupplier extenderMotorPositionSupplier, BooleanSupplier isScooperSpinningSupplier) {
        SmartDashboard.putNumber("Extender Motor Test Voltage", -1);
        SmartDashboard.putNumber("Scooper Motor Test Voltage", 12);
        SmartDashboard.putNumber("Pusher Motor Test Voltage", 4);
        SmartDashboard.putNumber("Minimum Scooper Speed", 0.1);
        SmartDashboard.putData("Intake Telemetry", builder -> {
            builder.addDoubleProperty("Extender Motor Position (revs)",
                () -> round(extenderMotorPositionSupplier.getAsDouble(), 3), null);
            builder.addBooleanProperty("Is Scooper Spinning", isScooperSpinningSupplier, null);
        });
    }

    static double getExtenderMotorTestVoltage() {
        return SmartDashboard.getNumber("Extender Motor Test Voltage", 0);
    }

    static double getScooperMotorTestVoltage() {
        return SmartDashboard.getNumber("Scooper Motor Test Voltage", 0);
    }

    static double getScooperMinSpeed() {
        return SmartDashboard.getNumber("Minimum Scooper Speed", 0);
    }

    static double getPusherMotorTestVoltage() {
        return SmartDashboard.getNumber("Pusher Motor Test Voltage", 0);
    }

    // =====================================
    // Controllers
    // =====================================
    static void setupController(SendableChooser<Controller> controllerChooser) {
        if (!Robot.IS_COMPETITION) SmartDashboard.putData("Controller Chooser", controllerChooser);
    }

    // =====================================
    // Commands: AimAtTarget
    // ===================================== // @formatter:off
    static void setupRotateToHub(Supplier<Pose2d> currPoseSupp, DoubleSupplier targetHeadingSupp, DoubleSupplier angDiffSupp) {
        SmartDashboard.putData("RotateToHub Telemetry", builder -> {
            builder.addDoubleProperty("Robot Rotation", () -> round(currPoseSupp.get().getRotation().getDegrees(), 1), null);
            builder.addDoubleProperty("Target Heading", () -> round(Math.toDegrees(targetHeadingSupp.getAsDouble()), 1), null);
            builder.addDoubleProperty("Angle Difference", () -> round(Math.toDegrees(angDiffSupp.getAsDouble()), 1), null);
        }); // @formatter:on
    }

    // =====================================
    // Commands: Automatically Reverse Scooper
    // ===================================== 
    static void putScooperReverseTimers() {
        SmartDashboard.putNumber("Scooper Auto Reverse/Loop Time", 0.2);
        SmartDashboard.putNumber("Scooper Auto Reverse/Reverse Delay", 0.3);
    }

    static double getScooperReverseLoopTime() {
        return SmartDashboard.getNumber("Scooper Auto Reverse/Loop Time", 0);
    }

    static double getScooperReverseDelay() {
        return SmartDashboard.getNumber("Scooper Auto Reverse/Reverse Delay", 0);
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
