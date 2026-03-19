package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class TargetHubOrHome extends Command {
    private final AimAtTarget aimAtHub;
    private final AimAtTarget aimAtLeftHome;
    private final AimAtTarget aimAtRightHome;

    public TargetHubOrHome(AimAtTarget aimAtHub, AimAtTarget aimAtLeftHome, AimAtTarget aimAtRightHome) {
        this.aimAtHub = aimAtHub;
        this.aimAtLeftHome = aimAtLeftHome;
        this.aimAtRightHome = aimAtRightHome;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}

    /** @return Translation2d of the right or left side of the home area to shoot at. */
    static Translation2d getClosestHome() {
        return Robot.isBlueAlliance ? Constants.BLUE_HUB_POSITION : Constants.RED_HUB_POSITION;
    }

}
