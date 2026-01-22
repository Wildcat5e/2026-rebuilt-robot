// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    // CHANGE MOTOR ID OBVIOUSLY
    // conveyor is motor that connects to actual wheels to intake fuel into storage
    private final TalonFX conveyorMotor = new TalonFX(0);
    // extender motor is motor that extends intake system outside of bumper
    private final TalonFX extenderMotor = new TalonFX(0);


    /** Creates a new Intake. */
    public Intake() {



    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command dropArm() {
        return new FunctionalCommand(null, null, null, null, null);
    }

}
