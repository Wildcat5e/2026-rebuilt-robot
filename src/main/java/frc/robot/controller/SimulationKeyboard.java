package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class SimulationKeyboard extends LogitechFlightStick {

    /** Uses {@link CommandJoystick} for Simulation Keyboard. @param port index on Driver Station */
    public SimulationKeyboard(int port) {
        super(port);
    }
}
