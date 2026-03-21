package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities;
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

    public ShootFuel(Flywheel flywheel, Hopper hopper, Intake intake, Drivetrain drivetrain) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        this.intake = intake;
        this.drivetrain = drivetrain;
        addRequirements(flywheel, hopper);
    }

    @Override
    public void initialize() {
        flywheelUpToSpeed = false;
    }

    @Override
    public void execute() {
        if (!flywheelUpToSpeed && flywheel.flywheelUpToSpeed()) {
            System.out.println(" UP TO SPEED ");
            flywheelUpToSpeed = true;
        }
        if (Utilities.inHome(drivetrain)) {
            flywheel.hubRunFlywheel();
            System.out.println("DYNAMIC !!!");
        } else {
            flywheel.homeRunFlywheel();
            System.out.println("STATIC !!!!");
        }
        if (flywheelUpToSpeed) {
            hopper.runHopper();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stopFlywheel();
        hopper.stopHopper();
        System.out.println("DONE");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
