package frc.robot.commands;

import static frc.robot.utilities.FieldUtils.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AimHandler extends Command {
    private final Drivetrain drivetrain;
    private final AimAtTarget aimAtHub;
    private final AimAtTarget aimAtUpperHome;
    private final AimAtTarget aimAtLowerHome;

    private Command activeCommand;

    public AimHandler(Drivetrain drivetrain, AimAtTarget aimAtHub, AimAtTarget aimAtUpperHome,
        AimAtTarget aimAtLowerHome) {
        this.drivetrain = drivetrain;
        this.aimAtHub = aimAtHub;
        this.aimAtUpperHome = aimAtUpperHome;
        this.aimAtLowerHome = aimAtLowerHome;
    }

    @Override
    public void initialize() {
        activeCommand = chooseCommand();
        if (activeCommand != null) activeCommand.initialize();
    }

    @Override
    public void execute() {
        Command newCommand = chooseCommand();

        if (newCommand != activeCommand) {
            if (activeCommand != null) activeCommand.end(true);
            activeCommand = newCommand;
            if (activeCommand != null) activeCommand.initialize();
        }

        if (activeCommand != null) {
            activeCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (activeCommand != null) activeCommand.end(interrupted);
    }

    /** @return The specific AimAtTarget command to run, or null if in the deadzone. */
    private Command chooseCommand() {
        if (inHome(drivetrain)) return aimAtHub;

        // Here, we have a deadzone between y = 3.75 y = 4.25, so the robot doesn't rotate there.
        // However, this is used by ShootingCalculator to find and set the ideal flywheel speed.
        // Thus, having a deadzone where we cannot shoot fuel could be catastrophic if our pose is incorrect.
        double yPosition = drivetrain.getState().Pose.getY();
        if (yPosition > 4.25) return aimAtUpperHome;
        if (yPosition < 3.75) return aimAtLowerHome;

        return null;
    }
}
