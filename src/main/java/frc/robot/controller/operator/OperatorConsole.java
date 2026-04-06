package frc.robot.controller.operator;

import static frc.robot.controller.operator.MacropadButtonsGenerated.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.DashboardManager;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DrivetrainUtils;

public class OperatorConsole {
    private final CommandGenericHID macropad = new CommandGenericHID(4);

    public OperatorConsole() {}

    // Helper method to convert the Enum into the 1-indexed WPILib button ID
    private int getBtn(MacropadButtonsGenerated action) {
        return action.ordinal() + 1;
    }

    private void bindWhileTrue(MacropadButtonsGenerated action, Command command) {
        macropad.button(getBtn(action)).whileTrue(command);
    }

    private void bindOnTrue(MacropadButtonsGenerated action, Command command) {
        macropad.button(getBtn(action)).onTrue(command);
    }

    private void bindDashboard(MacropadButtonsGenerated action, Runnable runnable) {
        macropad.button(getBtn(action)).onTrue(Commands.runOnce(runnable).ignoringDisable(true));
    }

        
    public void bindMacropad(RobotCommands commands, Flywheel flywheel, Intake intake, Hopper hopper, Drivetrain drivetrain) {
        // --- MACROPAD BINDINGS ---

        // LAYER 0 (No Modifiers)
        // @formatter:off
        bindWhileTrue(EXTENDER_POS, Commands.startEnd(() -> intake.setExtenderVoltagePositive(), () -> intake.stopExtender()));
        bindWhileTrue(EXTENDER_NEG, Commands.startEnd(() -> intake.setExtenderVoltageNegative(), () -> intake.stopExtender())); // @formatter:on
        bindDashboard(FLYWHEEL_MULT_UP, () -> DashboardManager.incrementFlywheelSpeedMultiplier(0.01));
        bindDashboard(FLYWHEEL_MULT_DOWN, () -> DashboardManager.incrementFlywheelSpeedMultiplier(-0.01));
        bindDashboard(TUNE_FLYWHEEL_UP, () -> DashboardManager.incrementTunableFlywheelSpeed(0.5));
        bindDashboard(TUNE_FLYWHEEL_DOWN, () -> DashboardManager.incrementTunableFlywheelSpeed(-0.5));
        bindWhileTrue(RUN_HOPPER, hopper.runHopperCommand());
        bindWhileTrue(RUN_HOPPER, intake.testPusher());
        bindWhileTrue(TUNABLE_FLYWHEEL, flywheel.tunableFlywheelSpeedCommand());
        bindWhileTrue(SWERVE_BRAKE, DrivetrainUtils.swerveDriveBrake(drivetrain));

        // LAYER 1 (Shift Held)
        bindWhileTrue(SCOOPER_REV, intake.reverseScooper());
        bindWhileTrue(PUSHER_REV, intake.reversePusher());
        bindWhileTrue(CONVEYOR_REV, hopper.reverseConveyor());
        bindWhileTrue(KICKER_REV, hopper.reverseKicker());
        bindWhileTrue(FLYWHEEL_REV, flywheel.reverseFlywheel());
        bindDashboard(HOME_FLYWHEEL_UP, () -> DashboardManager.incrementHomeFlywheelSpeedMultiplier(0.01));
        bindDashboard(HOME_FLYWHEEL_DOWN, () -> DashboardManager.incrementHomeFlywheelSpeedMultiplier(-0.01));

        // LAYER 2 (Control Held)
        bindWhileTrue(SCOOPER_FWD, intake.testScooper());
        bindWhileTrue(PUSHER_FWD, intake.testPusher());
        bindWhileTrue(CONVEYOR_FWD, hopper.testConveyor());
    }
}
