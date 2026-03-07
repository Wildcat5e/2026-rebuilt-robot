package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ShootingCalculator.ShotSolution;
import static frc.robot.Utilities.*;

public class Flywheel extends SubsystemBase {

    private final Drivetrain drivetrain;
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    private final double FLYWHEEL_RADIUS = 0.1; // Placeholder, in meters
    private final double FLYWHEEL_CIRCUMFERENCE = 2 * Math.PI * FLYWHEEL_RADIUS;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    private double currentFlywheelSpeed = 0;
    private double targetFlywheelSpeed = -1;

    public Flywheel(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        applyGearRatio(leftFlywheelMotor, 1);
        applyGearRatio(rightFlywheelMotor, 1);
    }

    private void setFlywheelMotorVoltages(double voltage) {
        leftFlywheelMotor.setVoltage(-voltage);
        rightFlywheelMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        currentFlywheelSpeed = getFlywheelSpeed();
    }

    public Command testSpinFlywheel() {
        return startEnd(() -> setFlywheelMotorVoltages(6), () -> setFlywheelMotorVoltages(0));
    }

    public Command testDynamicStartFlywheel() {
        return runEnd(() -> {
            // This code is run every 20 ms
            ShotSolution shotSolution = ShootingCalculator.calculate(drivetrain);
            targetFlywheelSpeed = shotSolution.flywheelSpeed();
            double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
            setFlywheelMotorVoltages(calculatedVoltage);
        },
            // on end
            () -> setFlywheelMotorVoltages(0));
    }

    public Command testStaticStartFlywheel() {
        return runEnd(() -> {
            double targetFlywheelSpeed = 3;
            double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
            setFlywheelMotorVoltages(calculatedVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    /**
     * Spins flywheel and calculates speed based on distance. Flywheel speed can be set to zero using another command
     */
    public void dynamicRunFlywheel() {
        ShotSolution shotSolution = ShootingCalculator.calculate(drivetrain);
        targetFlywheelSpeed = shotSolution.flywheelSpeed();
        double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
        setFlywheelMotorVoltages(calculatedVoltage);
    }

    /**
     * Starts flywheel with static speed for when the robot is in the middle or opposing alliance zone and shooting fuel
     * NOT in the hub
     */
    public void staticRunFlywheel() {
        targetFlywheelSpeed = 3;
        double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
        setFlywheelMotorVoltages(calculatedVoltage);
    }

    public void stopFlywheel() {
        setFlywheelMotorVoltages(0);
    }

    public boolean flywheelUpToSpeed() {
        return currentFlywheelSpeed > targetFlywheelSpeed * 0.9; // Placeholder
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

    /** Speed is in meters (of flywheel) per second */
    public double getFlywheelSpeed() {
        // Rotations Per Second (RPS) of flywheel
        double flywheelRps = leftFlywheelMotor.getVelocity().getValueAsDouble();
        return flywheelRps * FLYWHEEL_CIRCUMFERENCE;
    }

    // UNTESTED
    SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(voltage -> setFlywheelMotorVoltages(voltage.magnitude()), log -> {
            log.motor("flywheel-motor").voltage(leftFlywheelMotor.getMotorVoltage().getValue())
                .linearPosition(Distance.ofRelativeUnits(
                    leftFlywheelMotor.getPosition().getValueAsDouble() * FLYWHEEL_CIRCUMFERENCE, Meters))
                .linearVelocity(LinearVelocity.ofRelativeUnits(getFlywheelSpeed(), MetersPerSecond));
        }, this));

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
