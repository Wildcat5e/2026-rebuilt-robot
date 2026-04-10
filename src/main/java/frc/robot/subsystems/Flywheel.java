package frc.robot.subsystems;

import static frc.robot.utilities.TargetingUtils.*;
import static frc.robot.utilities.FieldUtils.*;
import static frc.robot.utilities.HardwareUtils.*;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.DashboardManager;

public class Flywheel extends SubsystemBase implements SysIdCapable {
    private final Drivetrain drivetrain;
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    // Old m/s based: ks: 0.025659, kv: 0.33677, ka: 0.040121
    final Slot0Configs feedforwardConfig = new Slot0Configs().withKS(0.033378).withKV(0.10787).withKA(0.014195);

    private double currentFlywheelSpeed = 0;
    private double targetFlywheelSpeed = 0;

    // 5 seconds * 50 loops per second = 250 samples
    private final LinearFilter speedFilter = LinearFilter.movingAverage(250);
    private double averageFlywheelSpeed = 0;
    private double calculatedVoltage = 0;

    private final SysIdRoutine routine =
        SysIdCapable.createAngularRoutine(this, rightFlywheelMotor, this::setFlywheelMotorVoltages);

    public Flywheel(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        setMotorClockwisePositive(leftFlywheelMotor);
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

    /** Spins flywheel at specified velocity. */
    public void setFlywheelVelocity(double targetFlywheelSpeed) {
        setVelocity(this.targetFlywheelSpeed = targetFlywheelSpeed, leftFlywheelMotor, rightFlywheelMotor);
    }

    public void stopFlywheel() {
        setFlywheelMotorVoltages(0);
    }

    /** @return revolutions per second */
    public double getFlywheelSpeed() {
        return rightFlywheelMotor.getVelocity().getValueAsDouble();
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

    public Command hubRunFlywheelCommand() {
        return runEnd(this::hubRunFlywheel, this::stopFlywheel);
    }

    public Command homeRunFlywheelCommand() {
        return runEnd(this::homeRunFlywheel, this::stopFlywheel);
    }

    /** Reads the "Flywheel Test Voltage" from SmartDashboard and applies it continuously. */
    public Command tunableFlywheelVoltageCommand() {
        return runEnd(() -> {
            double targetVoltage = DashboardManager.getFlywheelTestVoltage();
            setFlywheelMotorVoltages(targetVoltage);
        }, this::stopFlywheel);
    }

    public Command tunableFlywheelSpeedCommand() {
        return runEnd(() -> {
            setFlywheelVelocity(SmartDashboard.getNumber("Tunable Flywheel Speed", 0));
        }, this::stopFlywheel);
    }

    public Command reverseFlywheel() {
        return startEnd(() -> setFlywheelMotorVoltages(-12), this::stopFlywheel).withName("Reverse Flywheel");
    }

    public boolean isFlywheelUpToSpeed() {
        if (targetFlywheelSpeed == 0) return false;
        return currentFlywheelSpeed > targetFlywheelSpeed * 0.9;
    }

    /** Spins flywheel and calculates speed based on distance. */
    public void hubRunFlywheel() {
        // var shotSolution =
        //     ShootingCalculator.calculate(drivetrain, getHubPosition(), Constants.HUB_FLYWHEEL_SPEEDS_MAP);
        // targetFlywheelSpeed = shotSolution.flywheelSpeed();
        var speed = Constants.HUB_FLYWHEEL_SPEEDS_MAP.get(getTargetDistance(drivetrain, getHubPosition()))
            * DashboardManager.getFlywheelSpeedMultiplier();
        setFlywheelVelocity(speed);
    }

    /** Starts flywheel at constant speed for when the robot is shooting, but NOT into the hub. */
    public void homeRunFlywheel() {
        // var shotSolution =
        //     ShootingCalculator.calculate(drivetrain, getHomeTarget(drivetrain), Constants.HOME_FLYWHEEL_SPEEDS_MAP);
        // var speed = shotSolution.flywheelSpeed();
        var speed = Constants.HOME_FLYWHEEL_SPEEDS_MAP.get(getTargetDistance(drivetrain, getHomeTarget(drivetrain)))
            * DashboardManager.getHomeFlywheelSpeedMultiplier();
        setFlywheelVelocity(speed);
    }
}
