package frc.robot.controller;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Controller that will use the {@link Controller} selected by the dashboard widget. For example, you can select the
 * {@link Xbox} controller. Also, this silences unplugged controller warnings. You should probably directly use your
 * controller before competition instead of using this.
 */
public class MultiController extends Controller {
    private final SendableChooser<Controller> controllerChooser = new SendableChooser<Controller>();

    public MultiController() {
        DriverStation.silenceJoystickConnectionWarning(true);
        controllerChooser.setDefaultOption("Xbox Controller", new Xbox(0));
        controllerChooser.addOption("Logitech Flight Stick", new LogitechFlightStick(1));
        controllerChooser.addOption("Simulation Keyboard", new SimulationKeyboard(2));
        controllerChooser.addOption("8BitDo Controller", new Xbox(3));
        controllerChooser.addOption("Linux 8BitDo Controller", new Linux8BitDo(4));
        SmartDashboard.putData("Controller Chooser", controllerChooser);
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
    Trigger runIntake() {
        return new Trigger(() -> controllerChooser.getSelected().runIntake().getAsBoolean());
    }

    @Override
    Trigger shootFuel() {
        return new Trigger(() -> controllerChooser.getSelected().shootFuel().getAsBoolean());
    }

    @Override
    Trigger lowerIntake() {
        return new Trigger(() -> controllerChooser.getSelected().lowerIntake().getAsBoolean());
    }

    @Override
    Trigger raiseIntake() {
        return new Trigger(() -> controllerChooser.getSelected().raiseIntake().getAsBoolean());
    }

    @Override
    Trigger aimHandler() {
        return new Trigger(() -> controllerChooser.getSelected().aimHandler().getAsBoolean());
    }

    @Override
    Trigger manualFlywheel() {
        return new Trigger(() -> controllerChooser.getSelected().manualFlywheel().getAsBoolean());
    }

    @Override
    Trigger seedFieldCentric() {
        return new Trigger(() -> controllerChooser.getSelected().seedFieldCentric().getAsBoolean());
    }

    @Override
    Trigger reverse() {
        return new Trigger(() -> controllerChooser.getSelected().reverse().getAsBoolean());
    }

    @Override
    Trigger povUp() {
        return new Trigger(() -> controllerChooser.getSelected().povUp().getAsBoolean());
    }

    @Override
    Trigger povDown() {
        return new Trigger(() -> controllerChooser.getSelected().povDown().getAsBoolean());
    }

    @Override
    Trigger povLeft() {
        return new Trigger(() -> controllerChooser.getSelected().povLeft().getAsBoolean());
    }

    @Override
    Trigger povRight() {
        return new Trigger(() -> controllerChooser.getSelected().povRight().getAsBoolean());
    }

    @Override
    Trigger forwardSysIdQuasi() {
        return new Trigger(() -> controllerChooser.getSelected().forwardSysIdQuasi().getAsBoolean());
    }

    @Override
    Trigger backwardSysIdQuasi() {
        return new Trigger(() -> controllerChooser.getSelected().backwardSysIdQuasi().getAsBoolean());
    }

    @Override
    Trigger forwardSysIdDynamic() {
        return new Trigger(() -> controllerChooser.getSelected().forwardSysIdDynamic().getAsBoolean());
    }

    @Override
    Trigger backwardSysIdDynamic() {
        return new Trigger(() -> controllerChooser.getSelected().backwardSysIdDynamic().getAsBoolean());
    }
}
