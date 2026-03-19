package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DashboardManager;
import frc.robot.subsystems.ShootingCalculator.ShotSolution;
import static frc.robot.Utilities.*;

public class Flywheel extends SubsystemBase {

    private final Drivetrain drivetrain;
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    private final double FLYWHEEL_RADIUS = 0.0508;
    private final double FLYWHEEL_CIRCUMFERENCE = 2 * Math.PI * FLYWHEEL_RADIUS;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.025659, 0.33677, 0.040121);

    private double currentFlywheelSpeed = 0;
    private double targetFlywheelSpeed = 0;
    public double flywheelSpeedMult = 1.0;

    // 5 seconds * 50 loops per second = 250 samples
    private final LinearFilter speedFilter = LinearFilter.movingAverage(250);
    private double averageFlywheelSpeed = 0;
    private double calculatedVoltage = 0;

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

    /** Sets both flywheel motors to the specified voltage (left voltage negated). */
    private void setFlywheelMotorVoltages(double volts) {
        leftFlywheelMotor.setVoltage(-volts);
        rightFlywheelMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        currentFlywheelSpeed = getFlywheelSpeed();
        averageFlywheelSpeed = speedFilter.calculate(currentFlywheelSpeed);
        flywheelSpeedMult = DashboardManager.getFlywheelSpeedMultiplier();
    }

    /** Reads the "Flywheel Test Voltage" from SmartDashboard and applies it continuously. */
    public Command testTunableFlywheel() {
        return runEnd(() -> {
            double targetVoltage = DashboardManager.getFlywheelTestVoltage();
            setFlywheelMotorVoltages(targetVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    public Command testDynamicStartFlywheel() {
        return runEnd(() -> {
            // This code is run every 20 ms
            ShotSolution shotSolution = ShootingCalculator.calculate(drivetrain, flywheelSpeedMult);
            targetFlywheelSpeed = shotSolution.flywheelSpeed();
            // targetFlywheelSpeed = SmartDashboard.getNumber("Target Speed (m∕s)", 0);
            calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
            SmartDashboard.putNumber("Calculated Voltage", calculatedVoltage);
            setFlywheelMotorVoltages(calculatedVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    public Command testStaticStartFlywheel() {
        return runEnd(() -> {
            targetFlywheelSpeed = SmartDashboard.getNumber("Static Flywheel Speed", 0);
            double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
            setFlywheelMotorVoltages(calculatedVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    public Command reverseFlywheel() {
        return startEnd(() -> setFlywheelMotorVoltages(-3), () -> setFlywheelMotorVoltages(0))
            .withName("Reverse Flywheel");
    }

    public boolean flywheelUpToSpeed() {
        if (targetFlywheelSpeed == 0) return false;
        return currentFlywheelSpeed > targetFlywheelSpeed;
    }

    /** Spins flywheel and calculates speed based on distance. */
    public void dynamicRunFlywheel() {
        ShotSolution shotSolution = ShootingCalculator.calculate(drivetrain, flywheelSpeedMult);
        targetFlywheelSpeed = shotSolution.flywheelSpeed();
        double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
        setFlywheelMotorVoltages(calculatedVoltage);
    }

    /** Starts flywheel at constant speed for when the robot is shooting, but NOT into the hub. */
    public void staticRunFlywheel() {
        targetFlywheelSpeed = SmartDashboard.getNumber("Static Flywheel Speed", 0);
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

    /** Base command helper for flywheel levels in case PhotonVision breaks */
    private Command backupFlywheelCommandHelper(double distance) {
        return runEnd(() -> {
            targetFlywheelSpeed = ShootingCalculator.FLYWHEEL_SPEEDS_MAP.get(distance);
            double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
            setFlywheelMotorVoltages(calculatedVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    public void stopFlywheel() {
        setFlywheelMotorVoltages(0);
    }

    // final implementation should be a while true
    // public Command shootFuel() {
    //     return new ParallelCommandGroup(inHome(drivetrain) ? dynamicStartFlywheel() : staticStartFlywheel(),
    //         inHome(drivetrain) ? rotateToHub : Commands.none(),
    //         // could do something where you check the amount of motor ticks that have passed
    //         // to infer speed of flywheel instead of waiting time
    //         new SequentialCommandGroup(Commands.waitSeconds(1),
    //             new ParallelRaceGroup(testBothHoppers(), spinKicker())));
    // }

    /** @return meters per second */
    public double getFlywheelSpeed() {
        // Rotations Per Second (RPS) of flywheel
        double flywheelRps = rightFlywheelMotor.getVelocity().getValueAsDouble();
        return flywheelRps * FLYWHEEL_CIRCUMFERENCE;
    }

    // Untested
    SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(voltage -> setFlywheelMotorVoltages(voltage.magnitude()), log -> {
            log.motor("flywheel-motors").voltage(rightFlywheelMotor.getMotorVoltage().getValue())
                .linearPosition(Distance.ofRelativeUnits(
                    rightFlywheelMotor.getPosition().getValueAsDouble() * FLYWHEEL_CIRCUMFERENCE, Meters))
                .linearVelocity(LinearVelocity.ofRelativeUnits(getFlywheelSpeed(), MetersPerSecond));
        }, this));

    // --- sysId Flywheel FeedForward Calibration Tests ---

    public Command sysIdQuasistaticForward() {
        return routine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return routine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return routine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return routine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
