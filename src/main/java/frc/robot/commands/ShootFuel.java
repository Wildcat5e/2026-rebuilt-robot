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


    public ShootFuel(Flywheel flywheel, Hopper hopper) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        addRequirements(flywheel, hopper);
    }

    @Override
    public void initialize() {
        flywheel.setFlywheelMotorVoltages(3);
        hopper.setHopperVoltages(-8, -3);
    }

    @Override
    public void execute() {


    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setFlywheelMotorVoltages(0);
        hopper.setHopperVoltages(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
