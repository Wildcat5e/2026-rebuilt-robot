package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
        SmartDashboard.putData("Robot Telemetry", builder -> {
            builder.addDoubleProperty("Distance to Hub (m)", () -> round(getHubDistance(drivetrain), 3), null);
        });
        if (!Robot.IS_COMPETITION) {
            SmartDashboard.putData("RotateToHub PID Controller", RotateToHub.PID_CONTROLLER);
        }
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
        if (!Robot.IS_COMPETITION) {
            SmartDashboard.putNumber("Kicker Test Voltage", 8);
            SmartDashboard.putNumber("Conveyor Test Voltage", 1.0);
        }
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
    static void setupFlywheel(DoubleSupplier currentSpeedSupplier, DoubleSupplier targetSpeedSupplier,
        DoubleSupplier averageSpeedSupplier, DoubleSupplier calculatedVoltageSupplier) {
        SmartDashboard.putNumber("Flywheel Test Voltage", 5);
        SmartDashboard.putData("Flywheel Telemetry", builder -> {
            builder.addDoubleProperty("Current Speed (m\u2215s)", () -> round(currentSpeedSupplier.getAsDouble(), 3),
                null);
            builder.addDoubleProperty("5s Avg Speed (m\u2215s)", () -> round(averageSpeedSupplier.getAsDouble(), 3),
                null);
            builder.addDoubleProperty("Target Speed (m\u2215s)", () -> round(targetSpeedSupplier.getAsDouble(), 3),
                null);
            builder.addDoubleProperty("Calculated Voltage", () -> round(calculatedVoltageSupplier.getAsDouble(), 3),
                null);
        });
    }

    static double getFlywheelTestVoltage() {
        return SmartDashboard.getNumber("Flywheel Test Voltage", 0.0);
    }

    static double getFlywheelSpeedMultiplier() {
        return SmartDashboard.getNumber("Flywheel Speed Multiplier", 1.0);
    }

    static void incrementFlywheelSpeedMultiplier(double increment) {
        SmartDashboard.putNumber("Flywheel Speed Multiplier", getFlywheelSpeedMultiplier() + increment);
    }

    // =====================================
    // Subsystem: Intake 
    // =====================================

    static void setupIntake(DoubleSupplier extenderMotorPositionSupplier) {
        SmartDashboard.putNumber("Extender Motor Test Voltage", 3);
        SmartDashboard.putNumber("Scooper Motor Test Voltage", 3);
        SmartDashboard.putNumber("Pusher Motor Test Voltage", 3);
        SmartDashboard.putData("Intake Telemetry", builder -> {
            builder.addDoubleProperty("Extender Motor Position (revs)",
                () -> round(extenderMotorPositionSupplier.getAsDouble(), 3), null);
        });

        SmartDashboard.putNumber("Extender kP", 0);
        SmartDashboard.putNumber("Extender kI", 0);
        SmartDashboard.putNumber("Extender kD", 0);
        SmartDashboard.putNumber("Extender kS", 0);
        SmartDashboard.putNumber("Extender kV", 0);
        SmartDashboard.putNumber("Extender kG", 0);
        SmartDashboard.putNumber("Extender Motion Magic Cruise Velocity", 1);
        SmartDashboard.putNumber("Extender Motion Magic Acceleration", 2);
        SmartDashboard.putNumber("Extender Motion Magic Jerk", 0);
    }

    static double getExtenderkP() {
        return SmartDashboard.getNumber("Extender kP", 0);
    }

    static double getExtenderkI() {
        return SmartDashboard.getNumber("Extender kI", 0);
    }

    static double getExtenderkD() {
        return SmartDashboard.getNumber("Extender kD", 0);
    }

    static double getExtenderkS() {
        return SmartDashboard.getNumber("Extender kS", 0);
    }

    static double getExtenderkV() {
        return SmartDashboard.getNumber("Extender kV", 0);
    }

    static double getExtenderkG() {
        return SmartDashboard.getNumber("Extender kG", 0);
    }

    static double getExtenderMotionMagicCruiseVelocity() {
        return SmartDashboard.getNumber("Extender Motion Magic Cruise Velocity", 1);
    }

    static double getExtenderMotionMagicAcceleration() {
        return SmartDashboard.getNumber("Extender Motion Magic Acceleration", 2);
    }

    static double getExtenderMotionMagicJerk() {
        return SmartDashboard.getNumber("Extender Motion Magic Jerk", 0);
    }

    static double getExtenderMotorTestVoltage() {
        return SmartDashboard.getNumber("Extender Motor Test Voltage", 0);
    }

    static double getScooperMotorTestVoltage() {
        return SmartDashboard.getNumber("Scooper Motor Test Voltage", 0);
    }

    static double getPusherMotorTestVoltage() {
        return SmartDashboard.getNumber("Pusher Motor Test Voltage", 0);
    }

    // =====================================
    // Controllers
    // =====================================
    static void setupController(SendableChooser<Controller> controllerChooser) {
        if (!Robot.IS_COMPETITION) {
            SmartDashboard.putData("Controller Chooser", controllerChooser);
        }
    }

    // =====================================
    // Commands: RotateToHub
    // =====================================
    static void setupRotateToHub(BooleanSupplier useShootingCalculatorSupplier, Supplier<Pose2d> currentPoseSupplier,
        DoubleSupplier targetHeadingSupplier, DoubleSupplier angDiffSupplier) {
        SmartDashboard.putData("RotateToHub Telemetry", builder -> { // @formatter:off
            builder.addBooleanProperty("Enable Shooting Calculator",
                useShootingCalculatorSupplier, null);
            builder.addDoubleProperty("Robot Rotation",
                () -> round(currentPoseSupplier.get().getRotation().getDegrees(), 1), null);
            builder.addDoubleProperty("Target Heading",
                () -> round(Math.toDegrees(targetHeadingSupplier.getAsDouble()), 1), null);
            builder.addDoubleProperty("Angle Difference",
                () -> round(Math.toDegrees(angDiffSupplier.getAsDouble()), 1), null);
        }); // @formatter:on
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
