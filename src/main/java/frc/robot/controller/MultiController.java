package frc.robot.controller;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands;
import frc.robot.subsystems.Drivetrain;

/**
 * Controller that will use the {@link Controller} selected by the dashboard widget. For example, you can select the
 * {@link Xbox} controller. Also, this silences unplugged controller warnings. You should probably directly us your
 * controller before competition instead of using this.
 */
public class MultiController extends Controller {
    final SendableChooser<Controller> controllerChooser = new SendableChooser<Controller>();

    public MultiController(Drivetrain drivetrain, Commands commands) {
        DriverStation.silenceJoystickConnectionWarning(true);
        controllerChooser.setDefaultOption("Xbox Controller", new Xbox(0));
        controllerChooser.addOption("Logitech Flight Stick", new LogitechFlightStick(1));
        controllerChooser.addOption("Simulation Keyboard", new SimulationKeyboard(2));
        SmartDashboard.putData("Controller Chooser", controllerChooser);
        controllerChooser.onChange(controller -> bindingsSetup(drivetrain, commands));
        // Extra unneeded call just for good measure in case the sim GUI updates the chooser but Elastic with the actual robot doesn't.
        bindingsSetup(drivetrain, commands);
    }

    @Override
    double getRotation() {
        return controllerChooser.getSelected().getRotation();
    }

    @Override
    Translation2d getTranslation() {
        return controllerChooser.getSelected().getTranslation();
    }

    @Override
    Trigger activateIntake() {
        return controllerChooser.getSelected().activateIntake();
    }

    @Override
    Trigger shootFuel() {
        return controllerChooser.getSelected().shootFuel();
    }

    @Override
    Trigger rotateToHub() {
        return controllerChooser.getSelected().rotateToHub();
    }

    @Override
    Trigger lowerIntake() {
        return controllerChooser.getSelected().lowerIntake();
    }

    @Override
    Trigger raiseIntake() {
        return controllerChooser.getSelected().raiseIntake();
    }

    @Override
    Trigger povUp() {
        return controllerChooser.getSelected().povUp();
    }

    @Override
    Trigger povDown() {
        return controllerChooser.getSelected().povDown();
    }

    @Override
    Trigger povLeft() {
        return controllerChooser.getSelected().povLeft();
    }

    @Override
    Trigger povRight() {
        return controllerChooser.getSelected().povRight();
    }

    @Override
    Trigger forwardSysIdQuasi() {
        return controllerChooser.getSelected().forwardSysIdQuasi();
    }

    @Override
    Trigger backwardSysIdQuasi() {
        return controllerChooser.getSelected().backwardSysIdQuasi();
    }

    @Override
    Trigger forwardSysIdDynamic() {
        return controllerChooser.getSelected().forwardSysIdDynamic();
    }

    @Override
    Trigger backwardSysIdDynamic() {
        return controllerChooser.getSelected().backwardSysIdDynamic();
    }
}
