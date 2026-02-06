// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ShootFuel extends Command {
    Flywheel flywheel;
    Hopper hopper;
    Drivetrain drivetrain;

    /** Creates a new ShootFuel. */
    public ShootFuel(Flywheel flywheel, Hopper hopper, Drivetrain drivetrain) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        this.drivetrain = drivetrain;
        addRequirements(flywheel, hopper);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (Utilities.inHome(drivetrain)) {
            flywheel.dynamicRunFlywheel();
        } else {
            flywheel.staticRunFlywheel();
        }

        if (flywheel.flywheelUpToSpeed()) {
            hopper.runFeeder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stopFlywheel();
        hopper.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
