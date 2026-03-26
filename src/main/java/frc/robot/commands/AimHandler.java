package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.subsystems.Drivetrain;

public class AimHandler extends Command {
    private final Drivetrain drivetrain;
    private final AimAtTarget aimAtHub;
    private final AimAtTarget aimAtUpperHome;
    private final AimAtTarget aimAtLowerHome;

    private Command activeCommand;

    AimHandler(Drivetrain drivetrain, AimAtTarget aimAtHub, AimAtTarget aimAtUpperHome, AimAtTarget aimAtLowerHome) {
        this.drivetrain = drivetrain;
        this.aimAtHub = aimAtHub;
        this.aimAtUpperHome = aimAtUpperHome;
        this.aimAtLowerHome = aimAtLowerHome;
        addRequirements(drivetrain);
    }

    public AimHandler(Robot robot) {
        this(robot.drivetrain, AimAtTarget.hub(robot), AimAtTarget.upperHome(robot), AimAtTarget.lowerHome(robot));
    }

    @Override
    public void initialize() {
        activeCommand = chooseCommand();
        scheduleActive();
    }

    @Override
    public void execute() {
        Command newCommand = chooseCommand();

        if (newCommand != activeCommand) {
            cancelActive();

            activeCommand = newCommand;
            scheduleActive();
        }
    }

    @Override
    public void end(boolean interrupted) {
        cancelActive();
    }

    /**
     * @return The specific AimAtTarget command to run, or null if in the deadzone.
     */
    private Command chooseCommand() {
        if (Utilities.inHome(drivetrain)) {
            return aimAtHub;
        }
        if (drivetrain.getState().Pose.getY() > 4.25) {
            return aimAtUpperHome;
        }
        return aimAtLowerHome;
    }

    private void scheduleActive() {
        if (activeCommand != null) {
            CommandScheduler.getInstance().schedule(activeCommand);
        }
    }

    private void cancelActive() {
        if (activeCommand != null) {
            CommandScheduler.getInstance().cancel(activeCommand);
        }
    }
}
