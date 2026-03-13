package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.subsystems.Drivetrain;

public class TargetHubOrHome extends Command {
    private final Drivetrain drivetrain;
    private final AimAtTarget aimAtHub;
    private final AimAtTarget aimAtLeftHome;
    private final AimAtTarget aimAtRightHome;

    private Command activeCommand;

    public TargetHubOrHome(Drivetrain drivetrain, AimAtTarget aimAtHub, AimAtTarget aimAtLeftHome,
        AimAtTarget aimAtRightHome) {
        this.drivetrain = drivetrain;
        this.aimAtHub = aimAtHub;
        this.aimAtLeftHome = aimAtLeftHome;
        this.aimAtRightHome = aimAtRightHome;
    }

    @Override
    public void initialize() {
        activeCommand = null;

        if (Utilities.inHome(drivetrain)) {
            activeCommand = aimAtHub;
        } else {
            double yPosition = drivetrain.getState().Pose.getY();

            if (yPosition > 4.25) {
                activeCommand = Robot.isBlueAlliance ? aimAtLeftHome : aimAtRightHome;
            } else if (yPosition < 3.75) {
                activeCommand = Robot.isBlueAlliance ? aimAtRightHome : aimAtLeftHome;
            }
        }

        // Properly start the lifecycle of the chosen command
        if (activeCommand != null) {
            activeCommand.initialize();
        }
    }

    @Override
    public void execute() {
        if (activeCommand != null) {
            activeCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (activeCommand != null) {
            return activeCommand.isFinished();
        }

        // Return false so the command stays scheduled (but does nothing) if the button is held while in the deadzone.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (activeCommand != null) {
            activeCommand.end(interrupted);
        }
    }
}
