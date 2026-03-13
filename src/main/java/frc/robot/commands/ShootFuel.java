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

    public ShootFuel(Flywheel flywheel, Hopper hopper, Intake intake, Drivetrain drivetrain) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        this.intake = intake;
        this.drivetrain = drivetrain;
        addRequirements(flywheel, hopper);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (Utilities.inHome(drivetrain)) {
            flywheel.dynamicRunFlywheel();
            System.out.println("DYNAMIC !!!");
        } else {
            flywheel.staticRunFlywheel();
            System.out.println("STATIC !!!!");
        }

        if (flywheel.flywheelUpToSpeed()) {
            System.out.println(" UP TO SPEED ");
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
