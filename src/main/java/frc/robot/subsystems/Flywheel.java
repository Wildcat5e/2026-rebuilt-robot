package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utilities.FieldUtils.*;
import static frc.robot.utilities.HardwareUtils.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.DashboardManager;
import frc.robot.subsystems.ShootingCalculator.ShotSolution;
import static frc.robot.subsystems.SysIdCapable.*;

public class Flywheel extends SubsystemBase implements SysIdCapable {
    private final Drivetrain drivetrain;
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    private final double FLYWHEEL_RADIUS = 0.0508;
    private final double FLYWHEEL_CIRCUMFERENCE = 2 * Math.PI * FLYWHEEL_RADIUS;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.025659, 0.33677, 0.040121);

    private double currentFlywheelSpeed = 0;
    private double targetFlywheelSpeed = 0;
    private double flywheelSpeedMult = 1.0;

    // 5 seconds * 50 loops per second = 250 samples
    private final LinearFilter speedFilter = LinearFilter.movingAverage(250);
    private double averageFlywheelSpeed = 0;
    private double calculatedVoltage = 0;

    private final SysIdRoutine routine =
        createLinearRoutine(this, rightFlywheelMotor, this::setFlywheelMotorVoltages, FLYWHEEL_CIRCUMFERENCE);

    public Flywheel(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        applyGearRatio(leftFlywheelMotor, 1);
        applyGearRatio(rightFlywheelMotor, 1);
        // @formatter:off
        DashboardManager.setupFlywheel(
            () -> currentFlywheelSpeed,
            () -> targetFlywheelSpeed,
            () -> averageFlywheelSpeed,
            () -> calculatedVoltage); // @formatter:on
    }

    @Override
    public void periodic() {
        currentFlywheelSpeed = getFlywheelSpeed();
        averageFlywheelSpeed = speedFilter.calculate(currentFlywheelSpeed);
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return routine;
    }

    /** Spins flywheel and calculates speed based on distance. */
    public void hubRunFlywheel() {
        var shotSolution =
            ShootingCalculator.calculate(drivetrain, getHubPosition(), Constants.HUB_FLYWHEEL_SPEEDS_MAP);
        targetFlywheelSpeed = shotSolution.flywheelSpeed();
        double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
        setFlywheelMotorVoltages(calculatedVoltage);
    }

    public Command hubRunFlywheelCommand() {
        return runEnd(() -> {
            // This code is run every 20 ms
            var shotSolution =
                ShootingCalculator.calculate(drivetrain, getHubPosition(), Constants.HUB_FLYWHEEL_SPEEDS_MAP);
            targetFlywheelSpeed = shotSolution.flywheelSpeed();
            // targetFlywheelSpeed = SmartDashboard.getNumber("Target Speed (m∕s)", 0);
            calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
            SmartDashboard.putNumber("Calculated Voltage", calculatedVoltage);
            setFlywheelMotorVoltages(calculatedVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    /** Starts flywheel for when the robot is shooting, but NOT into the hub. Rather, at the Alliance Home. */
    public void homeRunFlywheel() {
        var shotSolution =
            ShootingCalculator.calculate(drivetrain, getHomeTarget(drivetrain), Constants.HOME_FLYWHEEL_SPEEDS_MAP);
        targetFlywheelSpeed = shotSolution.flywheelSpeed();
        double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
        setFlywheelMotorVoltages(calculatedVoltage);
    }

    public Command backupFlywheelL1() {
        return backupFlywheelCommandHelper(2.0);
    }

    public Command backupFlywheelL2() {
        return backupFlywheelCommandHelper(3.5);
    }

    public Command backupFlywheelL3() {
        return backupFlywheelCommandHelper(5.0);
    }

    public Command reverseFlywheel() {
        return startEnd(() -> setFlywheelMotorVoltages(-3), () -> setFlywheelMotorVoltages(0))
            .withName("Reverse Flywheel");
    }

    public void stopFlywheel() {
        setFlywheelMotorVoltages(0);
    }

    public Command tunableFlywheelSpeedCommand() {
        return runEnd(() -> {
            targetFlywheelSpeed = DashboardManager.getTunableFlywheelSpeed();
            double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
            setFlywheelMotorVoltages(calculatedVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    public Command tunableFlywheelVoltageCommand() {
        return runEnd(() -> {
            double targetVoltage = DashboardManager.getFlywheelTestVoltage();
            setFlywheelMotorVoltages(targetVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    /** Spins flywheel at specified speed in m/s. */
    public void setFlywheelSpeed(double targetSpeed) {
        double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetSpeed);
        setFlywheelMotorVoltages(calculatedVoltage);
    }

    /** Sets both flywheel motors to the specified voltage (left voltage negated). */
    private void setFlywheelMotorVoltages(double volts) {
        leftFlywheelMotor.setVoltage(-volts);
        rightFlywheelMotor.setVoltage(volts);
    }

    public boolean isFlywheelUpToSpeed() {
        if (targetFlywheelSpeed < 0.1) return false;
        return currentFlywheelSpeed > targetFlywheelSpeed;
    }

    /** @return meters per second */
    public double getFlywheelSpeed() {
        // Rotations Per Second (RPS) of flywheel
        double flywheelRps = rightFlywheelMotor.getVelocity().getValueAsDouble();
        return flywheelRps * FLYWHEEL_CIRCUMFERENCE;
    }

    /** Base command helper for flywheel levels in case PhotonVision breaks */
    private Command backupFlywheelCommandHelper(double distance) {
        return runEnd(() -> {
            targetFlywheelSpeed = Constants.HUB_FLYWHEEL_SPEEDS_MAP.get(distance);
            double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
            setFlywheelMotorVoltages(calculatedVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }
}
