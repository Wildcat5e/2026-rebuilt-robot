package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Utilities.*;
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

    private final TalonFX flywheelMotor = new TalonFX(0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
    // UPDATE GEAR RATIO, CURRENTLY A PLACEHOLDER
    // FLYWHEEL RADIUS PLACEHOLDER, NEEDS TO BE IN METERS
    double FLYWHEEL_RADIUS = 3;
    double FLYWHEEL_CIRCUMFERENCE = 2 * Math.PI * FLYWHEEL_RADIUS;
    double currentVelocity;
    double GEAR_RATIO = 0.5;
    double previousRotation = 0;
    double currentRotation = 0;
    double rotationDifference = 0;
    double deltaTime = 0.02;
    double currentFlywheelSpeed = 0;
    double targetFlywheelSpeed;

    /** Creates a new Outtake. */
    public Flywheel(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        currentFlywheelSpeed = getFlywheelSpeed();
    }

    public Command testSpinFlywheel() {
        return startEnd(() -> flywheelMotor.setVoltage(3), () -> flywheelMotor.setVoltage(0));
    }

    public Command testDynamicStartFlywheel() {
        return runEnd(() -> {
            // all this code is ran every 20 ms
            ShotSolution shotSolution = ShootingCalculator.calculate(drivetrain);
            double calculatedFlywheelSpeed = shotSolution.flywheelSpeed();
            double calculatedVoltage =
                feedforward.calculateWithVelocities(currentFlywheelSpeed, calculatedFlywheelSpeed);
            flywheelMotor.setVoltage(calculatedVoltage);
        },
            // on end
            () -> flywheelMotor.setVoltage(0));
    }

    public Command testStaticStartFlywheel() {
        return runEnd(() -> {
            double targetFlywheelSpeed = 3;
            double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
            flywheelMotor.setVoltage(calculatedVoltage);
        }, () -> flywheelMotor.setVoltage(0));
    }

    /**
     * Starts flywheel and calculates speed based on distance, you need to set speed of flywheel to 0 with another
     * command
     */
    public void dynamicRunFlywheel() {
        ShotSolution shotSolution = ShootingCalculator.calculate(drivetrain);
        targetFlywheelSpeed = shotSolution.flywheelSpeed();
        double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
        flywheelMotor.setVoltage(calculatedVoltage);
    }

    //** Starts flywheel with static speed, for when the robot is in the middle or opposing alliance zone and shooting fuel NOT in the hub */
    public void staticRunFlywheel() {
        targetFlywheelSpeed = 3;
        double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, targetFlywheelSpeed);
        flywheelMotor.setVoltage(calculatedVoltage);
    }

    public void stopFlywheel() {
        flywheelMotor.setVoltage(0);
    }

    public boolean flywheelUpToSpeed() {
        // PLACE HOLDER VALUE
        return currentFlywheelSpeed > targetFlywheelSpeed * 0.9;
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
        // need to make sure rotation of motor starts at 0
        currentRotation = flywheelMotor.getPosition().getValueAsDouble() * GEAR_RATIO;
        rotationDifference = Math.abs(currentRotation - previousRotation);
        // meters per second of flywheel (linear)
        currentFlywheelSpeed = (rotationDifference / deltaTime) * FLYWHEEL_CIRCUMFERENCE;

        previousRotation = currentRotation;
        return currentFlywheelSpeed;
    }

    // UNTESTED
    SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(voltage -> flywheelMotor.setVoltage(voltage.magnitude()), log -> {
            log.motor("flywheel-motor").voltage(flywheelMotor.getMotorVoltage().getValue())
                .linearPosition(Distance.ofRelativeUnits(
                    flywheelMotor.getPosition().getValueAsDouble() * GEAR_RATIO * FLYWHEEL_CIRCUMFERENCE, Meters))
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
