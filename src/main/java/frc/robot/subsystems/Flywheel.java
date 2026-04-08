package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utilities.TargetingUtils.*;
import static frc.robot.utilities.FieldUtils.*;
import static frc.robot.utilities.HardwareUtils.*;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.DashboardManager;

public class Flywheel extends SubsystemBase {
    private final Drivetrain drivetrain;
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    private final double FLYWHEEL_RADIUS = 0.0508;
    private final double FLYWHEEL_CIRCUMFERENCE = 2 * Math.PI * FLYWHEEL_RADIUS;
    final Slot0Configs feedforwardConfig = new Slot0Configs().withKS(0.025659).withKV(0.33677).withKA(0.040121);

    private double currentFlywheelSpeed = 0;
    private double targetFlywheelSpeed = 0;

    // 5 seconds * 50 loops per second = 250 samples
    private final LinearFilter speedFilter = LinearFilter.movingAverage(250);
    private double averageFlywheelSpeed = 0;
    private double calculatedVoltage = 0;

    public Flywheel(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        invertMotor(leftFlywheelMotor);
        applyGearRatio(1, leftFlywheelMotor, rightFlywheelMotor);
        applyFeedforward(feedforwardConfig, leftFlywheelMotor, rightFlywheelMotor);
        // @formatter:off
        DashboardManager.setupFlywheel(
            () -> currentFlywheelSpeed,
            () -> targetFlywheelSpeed,
            () -> averageFlywheelSpeed,
            () -> calculatedVoltage); // @formatter:on
    }

    /** Sets both flywheel motors to the specified voltage (left voltage negated in constructor). */
    private void setFlywheelMotorVoltages(double volts) {
        leftFlywheelMotor.setVoltage(volts);
        rightFlywheelMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        currentFlywheelSpeed = getFlywheelSpeed();
        averageFlywheelSpeed = speedFilter.calculate(currentFlywheelSpeed);
    }

    /** Reads the "Flywheel Test Voltage" from SmartDashboard and applies it continuously. */
    public Command tunableFlywheelVoltageCommand() {
        return runEnd(() -> {
            double targetVoltage = DashboardManager.getFlywheelTestVoltage();
            setFlywheelMotorVoltages(targetVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    public Command hubRunFlywheelCommand() {
        return runEnd(() -> {
            var shotSolution =
                ShootingCalculator.calculate(drivetrain, getHubPosition(), Constants.HUB_FLYWHEEL_SPEEDS_MAP);
            targetFlywheelSpeed = shotSolution.flywheelSpeed();
            setVelocity(targetFlywheelSpeed, leftFlywheelMotor, rightFlywheelMotor);
        }, () -> setFlywheelMotorVoltages(0));
    }

    public Command tunableFlywheelSpeedCommand() {
        return runEnd(() -> {
            targetFlywheelSpeed = SmartDashboard.getNumber("Tunable Flywheel Speed", 0);
            setVelocity(targetFlywheelSpeed, leftFlywheelMotor, rightFlywheelMotor);
        }, () -> setFlywheelMotorVoltages(0));
    }

    public Command reverseFlywheel() {
        return startEnd(() -> setFlywheelMotorVoltages(-12), () -> setFlywheelMotorVoltages(0))
            .withName("Reverse Flywheel");
    }

    public boolean isFlywheelUpToSpeed() {
        if (targetFlywheelSpeed == 0) return false;
        return currentFlywheelSpeed > targetFlywheelSpeed;
    }

    /** Spins flywheel and calculates speed based on distance. */
    public void hubRunFlywheel() {
        // var shotSolution =
        //     ShootingCalculator.calculate(drivetrain, getHubPosition(), Constants.HUB_FLYWHEEL_SPEEDS_MAP);
        // targetFlywheelSpeed = shotSolution.flywheelSpeed();
        targetFlywheelSpeed = Constants.HUB_FLYWHEEL_SPEEDS_MAP.get(getTargetDistance(drivetrain, getHubPosition()))
            * DashboardManager.getFlywheelSpeedMultiplier();
        setVelocity(targetFlywheelSpeed, leftFlywheelMotor, rightFlywheelMotor);
    }

    /** Spins flywheel at specified speed. */
    public void setFlywheelSpeed(double targetFlywheelSpeed) {
        setVelocity(this.targetFlywheelSpeed = targetFlywheelSpeed, leftFlywheelMotor, rightFlywheelMotor);
    }

    /** Starts flywheel at constant speed for when the robot is shooting, but NOT into the hub. */
    public void homeRunFlywheel() {
        var shotSolution =
            ShootingCalculator.calculate(drivetrain, getHomeTarget(drivetrain), Constants.HOME_FLYWHEEL_SPEEDS_MAP);
        targetFlywheelSpeed = shotSolution.flywheelSpeed();
        targetFlywheelSpeed =
            Constants.HOME_FLYWHEEL_SPEEDS_MAP.get(getTargetDistance(drivetrain, getHomeTarget(drivetrain)))
                * DashboardManager.getHomeFlywheelSpeedMultiplier();
        // double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
        // setFlywheelMotorVoltages(calculatedVoltage);
        setVelocity(targetFlywheelSpeed, leftFlywheelMotor, rightFlywheelMotor);
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
