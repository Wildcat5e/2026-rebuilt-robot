package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.DashboardManager;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class OperatorConsole {
    private final CommandGenericHID macropad = new CommandGenericHID(4);

    public OperatorConsole() {}

    public void bindMacropad(RobotCommands commands, Flywheel flywheel, Intake intake, Hopper hopper) {
        // --- MACROPAD BINDINGS ---
        // LAYER 0 (No Modifiers)
        macropad.button(1).whileTrue(
            Commands.startEnd(() -> intake.setExtenderVoltagePositive(), () -> intake.setExtenderVoltageZero()));
        macropad.button(2).whileTrue(
            Commands.startEnd(() -> intake.setExtenderVoltageNegative(), () -> intake.setExtenderVoltageZero()));
        macropad.button(3).onTrue(
            Commands.runOnce(() -> DashboardManager.incrementFlywheelSpeedMultiplier(0.01)).ignoringDisable(true));
        macropad.button(4).onTrue(
            Commands.runOnce(() -> DashboardManager.incrementFlywheelSpeedMultiplier(-0.01)).ignoringDisable(true));
        macropad.button(5)
            .onTrue(Commands.runOnce(() -> DashboardManager.incrementTunableFlywheelSpeed(0.5)).ignoringDisable(true));
        macropad.button(6)
            .onTrue(Commands.runOnce(() -> DashboardManager.incrementTunableFlywheelSpeed(-0.5)).ignoringDisable(true));
        macropad.button(7).whileTrue(hopper.testTunableKicker());
        macropad.button(8).whileTrue(flywheel.tunableFlywheelSpeedCommand());

        // LAYER 1 (Shift Held)
        macropad.button(9).whileTrue(intake.reverseScooper());
        macropad.button(10).whileTrue(intake.reversePusher());
        macropad.button(11).whileTrue(hopper.reverseConveyor());
        macropad.button(12).whileTrue(hopper.reverseKicker());
        macropad.button(13).whileTrue(flywheel.reverseFlywheel());

        // LAYER 2 (Control Held)
        macropad.button(14).whileTrue(intake.testScooper());
        macropad.button(15).whileTrue(intake.testPusher());
        macropad.button(16).whileTrue(hopper.testConveyor());
    }
}
