package frc.robot.commands;

import static frc.robot.utilities.FieldUtils.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class ShootFuel extends Command {
    Flywheel flywheel;
    Hopper hopper;
    Intake intake;
    Drivetrain drivetrain;
    boolean flywheelUpToSpeed;

    public ShootFuel(Flywheel flywheel, Hopper hopper, Drivetrain drivetrain) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        this.drivetrain = drivetrain;
        addRequirements(flywheel, hopper);
    }

    @Override
    public void initialize() {
        flywheelUpToSpeed = false;
    }

    @Override
    public void execute() {
        if (!flywheelUpToSpeed && flywheel.isFlywheelUpToSpeed()) {
            System.out.println("FLYWHEEL UP TO SPEED");
            flywheelUpToSpeed = true;
        }
        if (inHome(drivetrain)) {
            flywheel.hubRunFlywheel();
        } else {
            flywheel.homeRunFlywheel();
        }
        if (flywheelUpToSpeed) {
            hopper.runHopper();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stopFlywheel();
        hopper.stopHopper();
        System.out.println("DONE SHOOTING");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
