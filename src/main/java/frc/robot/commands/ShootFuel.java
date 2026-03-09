package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utilities;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

public class ShootFuel extends Command {
    Flywheel flywheel;
    Hopper hopper;
    Drivetrain drivetrain;

    public ShootFuel(Flywheel flywheel, Hopper hopper, Drivetrain drivetrain) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        this.drivetrain = drivetrain;
        addRequirements(flywheel, hopper);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        hopper.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
