package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ShootingCalculator.ShotSolution;

public class Flywheel extends SubsystemBase {

    private final Drivetrain drivetrain;
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    private final double FLYWHEEL_RADIUS = 0.1; // Placeholder, in meters
    private final double FLYWHEEL_CIRCUMFERENCE = 2 * Math.PI * FLYWHEEL_RADIUS;
    private final double GEAR_RATIO = 0.5; // Placeholder. We should not need a gear ratio if we use in-built methods and set TalonFXConfiguration.Feedback.SensorToMechanismRatio
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    private double currentFlywheelSpeed = 0;
    private double targetFlywheelSpeed;

    public Flywheel(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        SmartDashboard.putNumber("Flywheel Test Voltage", 5);
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

    /**
     * Reads the "Flywheel Test Voltage" from SmartDashboard and applies it continuously.
     */
    public Command testTunableFlywheel() {
        return runEnd(() -> {
            // Fetch the current number from the dashboard (defaults to 0.0 if not found)
            double targetVoltage = SmartDashboard.getNumber("Flywheel Test Voltage", 0.0);
            setFlywheelMotorVoltages(targetVoltage);
        }, () -> setFlywheelMotorVoltages(0));
    }

    public Command testDynamicStartFlywheel() {
        return runEnd(() -> {
            // This code is run every 20 ms
            ShotSolution shotSolution = ShootingCalculator.calculate(drivetrain);
            double calculatedFlywheelSpeed = shotSolution.flywheelSpeed();
            double calculatedVoltage =
                feedforward.calculateWithVelocities(currentFlywheelSpeed, calculatedFlywheelSpeed);
            setFlywheelMotorVoltages(calculatedVoltage);
        }, () -> setFlywheelMotorVoltages(0));
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
        // Rotations Per Second (RPS) of motor rotor
        double motorRps = leftFlywheelMotor.getVelocity().getValueAsDouble();

        // Convert RPS to linear meters per second
        return motorRps * GEAR_RATIO * FLYWHEEL_CIRCUMFERENCE;
    }

    // UNTESTED
    SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(voltage -> setFlywheelMotorVoltages(voltage.magnitude()), log -> {
            log.motor("flywheel-motor").voltage(leftFlywheelMotor.getMotorVoltage().getValue())
                .linearPosition(Distance.ofRelativeUnits(
                    leftFlywheelMotor.getPosition().getValueAsDouble() * GEAR_RATIO * FLYWHEEL_CIRCUMFERENCE, Meters))
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
