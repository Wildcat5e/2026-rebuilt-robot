package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class FeedShooter extends Command {
    Flywheel flywheel;
    Hopper hopper;
    Intake intake;
    Drivetrain drivetrain;
    boolean flywheelUpToSpeed;

    public FeedShooter(Flywheel flywheel, Hopper hopper, Drivetrain drivetrain) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        this.drivetrain = drivetrain;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        flywheelUpToSpeed = false;
    }

    @Override
    public void execute() {
        if (!flywheelUpToSpeed && flywheel.isFlywheelUpToSpeed()) {
            System.out.println(" UP TO SPEED "); // debug
            flywheelUpToSpeed = true;
        }
        if (flywheelUpToSpeed) {
            hopper.runHopper();
        }
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stopHopper();
    }
}
