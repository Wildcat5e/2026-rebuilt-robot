// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final TalonFX leftHopperMotor = new TalonFX(0);
    private final TalonFX rightHopperMotor = new TalonFX(0);
    private final TalonFX kickerMotor = new TalonFX(0);

    /** Creates a new Storage. */
    public Hopper() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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

    public Command testSpinKicker() {
        return startEnd(() -> kickerMotor.setVoltage(3), () -> kickerMotor.setVoltage(0));
    }

    public void runFeeder() {
        leftHopperMotor.setVoltage(3);
        rightHopperMotor.setVoltage(-3);
        kickerMotor.setVoltage(3);
    }

    public void stopFeeder() {
        leftHopperMotor.setVoltage(0);
        rightHopperMotor.setVoltage(-0);
        kickerMotor.setVoltage(0);
    }
}
