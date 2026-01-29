package frc.robot.subsystems;

import static frc.robot.Utilities.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {

    private final TalonFX leftHopperMotor = new TalonFX(0);
    private final TalonFX rightHopperMotor = new TalonFX(0);
    private final TalonFX kickerMotor = new TalonFX(0);
    private final TalonFX flywheelMotor = new TalonFX(0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
    double GEAR_RATIO = 0;
    double previousRotation = 0;
    double currentRotation = 0;
    double rotationDifference = 0;
    double deltaTime = 0;
    double speed = 0;

    /** Creates a new Outtake. */
    public Outtake() {
        feedforward.calculateWithVelocities(0, 0);
    }

    @Override
    public void periodic() {
        // need to make sure rotation of motor starts at 0
        currentRotation = flywheelMotor.getPosition().getValueAsDouble();
        rotationDifference = Math.abs(currentRotation - previousRotation);

    }

    public Command testLeftHopper() {
        // might have to change voltage signs, left and right
        // motor should spin in different directions
        return startEnd(() -> leftHopperMotor.setVoltage(3), () -> leftHopperMotor.setVoltage(0));
    }

    public Command testRightHopper() {
        return startEnd(() -> rightHopperMotor.setVoltage(-3), () -> rightHopperMotor.setVoltage(0));
    }

    public Command testBothHoppers() {
        return startEnd(() -> {
            leftHopperMotor.setVoltage(3);
            rightHopperMotor.setVoltage(-3);
        }, () -> {
            leftHopperMotor.setVoltage(0);
            rightHopperMotor.setVoltage(0);
        });
    }

    public Command spinKicker() {
        return startEnd(() -> kickerMotor.setVoltage(3), () -> kickerMotor.setVoltage(0));
    }

    public Command spinFlywheel() {
        return startEnd(() -> flywheelMotor.setVoltage(3), () -> flywheelMotor.setVoltage(0));
    }

    /**
     * Starts flywheel and calculates speed based on distance, you need to set speed of flywheel to 0 with another
     * command
     */
    public Command startFlywheel() {
        return runEnd(() -> {
            // NEED TO MAKE A FUNCTION TO CALCULATE CURRENT FLYWHEEL SPEED
            double currentFlywheelSpeed = 0;
            double distance = getHubDistance();
            // all placeholder values and formula
            double calculatedVelocity = 0 * distance + 1;
            double calculatedVoltage = feedforward.calculateWithVelocities(currentFlywheelSpeed, calculatedVelocity);
            flywheelMotor.setVoltage(calculatedVoltage);
        }, () -> flywheelMotor.setVoltage(0));
    }


    // final implementation should be a while true
    public Command shootFuel() {
        return new ParallelCommandGroup(startFlywheel(),
            // could do something where you check the amount of motor ticks that have passed
            // to infer speed of flywheel instead of waiting time
            new SequentialCommandGroup(Commands.waitSeconds(1),
                new ParallelRaceGroup(testBothHoppers(), spinKicker())));
    }
}
