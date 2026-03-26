package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

import java.util.function.BooleanSupplier;

public class ShootFuel extends Command {
    Flywheel flywheel;
    Hopper hopper;
    BooleanSupplier inHome;

    boolean flywheelUpToSpeed;

    public ShootFuel(Flywheel flywheel, Hopper hopper, BooleanSupplier inHome) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        this.inHome = inHome;
        addRequirements(flywheel, hopper);
    }

    @Override
    public void initialize() {
        flywheelUpToSpeed = false;
    }

    @Override
    public void execute() {
        if (inHome.getAsBoolean()) {
            flywheel.homeRunFlywheel();
        } else {
            flywheel.hubRunFlywheel();
        }
        if (!flywheelUpToSpeed && flywheel.flywheelUpToSpeed()) {
            flywheelUpToSpeed = true;
        }
        if (flywheelUpToSpeed) {
            hopper.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stopFlywheel();
        hopper.stop();
    }
}
