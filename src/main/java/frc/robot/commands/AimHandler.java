package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.subsystems.Drivetrain;

public class AimHandler extends Command {
    private final Drivetrain drivetrain;
    private final AimAtTarget aimAtHub;
    private final AimAtTarget aimAtLeftHome;
    private final AimAtTarget aimAtRightHome;

    private Command activeCommand;

    public AimHandler(Drivetrain drivetrain, AimAtTarget aimAtHub, AimAtTarget aimAtLeftHome,
        AimAtTarget aimAtRightHome) {
        this.drivetrain = drivetrain;
        this.aimAtHub = aimAtHub;
        this.aimAtLeftHome = aimAtLeftHome;
        this.aimAtRightHome = aimAtRightHome;
    }

    @Override
    public void initialize() {
        activeCommand = chooseCommand();
        if (activeCommand != null) {
            activeCommand.initialize();
        }
    }

    @Override
    public void execute() {
        Command newCommand = chooseCommand();

        if (newCommand != activeCommand) {
            if (activeCommand != null) {
                activeCommand.end(true);
            }

            activeCommand = newCommand;
            if (activeCommand != null) {
                activeCommand.initialize();
            }
        }

        if (activeCommand != null) {
            activeCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (activeCommand != null) {
            activeCommand.end(interrupted);
        }
    }

    /** @return The specific AimAtTarget command to run, or null if in the deadzone. */
    private Command chooseCommand() {
        if (Utilities.inHome(drivetrain)) {
            return aimAtHub;
        }

        double yPosition = drivetrain.getState().Pose.getY();
        if (yPosition > 4.25) {
            return Robot.isBlueAlliance ? aimAtLeftHome : aimAtRightHome;
        } else if (yPosition < 3.75) {
            return Robot.isBlueAlliance ? aimAtRightHome : aimAtLeftHome;
        }

        return null;
    }
}
