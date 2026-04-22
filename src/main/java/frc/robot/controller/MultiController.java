package frc.robot.controller;

import java.util.function.Function;
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
    private final SendableChooser<Controller> controllerChooser = new SendableChooser<>();

    public MultiController() {
        DriverStation.silenceJoystickConnectionWarning(true);
        controllerChooser.setDefaultOption("Xbox Controller", new Xbox(0));
        controllerChooser.addOption("Logitech Flight Stick", new LogitechFlightStick(1));
        controllerChooser.addOption("Simulation Keyboard", new SimulationKeyboard(2));
        controllerChooser.addOption("8BitDo Controller", new Xbox(3));
        controllerChooser.addOption("Linux 8BitDo Controller", new Linux8BitDo(4));
        SmartDashboard.putData("Controller Chooser", controllerChooser);
    }

    /** Helper method to cleanly extract the active Trigger based on the selected controller. */
    private Trigger getDynamicTrigger(Function<Controller, Trigger> triggerSelector) {
        return new Trigger(() -> triggerSelector.apply(controllerChooser.getSelected()).getAsBoolean());
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
        return getDynamicTrigger(Controller::runIntake);
    }

    @Override
    Trigger shootFuel() {
        return getDynamicTrigger(Controller::shootFuel);
    }

    @Override
    Trigger lowerIntake() {
        return getDynamicTrigger(Controller::lowerIntake);
    }

    @Override
    Trigger raiseIntake() {
        return getDynamicTrigger(Controller::raiseIntake);
    }

    @Override
    Trigger aimHandler() {
        return getDynamicTrigger(Controller::aimHandler);
    }

    @Override
    Trigger manualFlywheel() {
        return getDynamicTrigger(Controller::manualFlywheel);
    }

    @Override
    Trigger seedFieldCentric() {
        return getDynamicTrigger(Controller::seedFieldCentric);
    }

    @Override
    Trigger reverse() {
        return getDynamicTrigger(Controller::reverse);
    }

    @Override
    Trigger trenchPath() {
        return getDynamicTrigger(Controller::trenchPath);
    }

    @Override
    Trigger povUp() {
        return getDynamicTrigger(Controller::povUp);
    }

    @Override
    Trigger povDown() {
        return getDynamicTrigger(Controller::povDown);
    }

    @Override
    Trigger povLeft() {
        return getDynamicTrigger(Controller::povLeft);
    }

    @Override
    Trigger povRight() {
        return getDynamicTrigger(Controller::povRight);
    }

    @Override
    Trigger forwardSysIdQuasi() {
        return getDynamicTrigger(Controller::forwardSysIdQuasi);
    }

    @Override
    Trigger backwardSysIdQuasi() {
        return getDynamicTrigger(Controller::backwardSysIdQuasi);
    }

    @Override
    Trigger forwardSysIdDynamic() {
        return getDynamicTrigger(Controller::forwardSysIdDynamic);
    }

    @Override
    Trigger backwardSysIdDynamic() {
        return getDynamicTrigger(Controller::backwardSysIdDynamic);
    }
}
