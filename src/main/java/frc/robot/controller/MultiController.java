package frc.robot.controller;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DashboardManager;

/**
 * Controller that will use the {@link Controller} selected by the dashboard widget. For example, you can select the
 * {@link Xbox} controller. Also, this silences unplugged controller warnings. You should probably directly use your
 * controller before competition instead of using this.
 */
public class MultiController extends Controller {
    private final SendableChooser<Controller> controllerChooser = new SendableChooser<>();
    private final Xbox defaultXbox = new Xbox(0);

    public MultiController() {
        DriverStation.silenceJoystickConnectionWarning(true);
        controllerChooser.setDefaultOption("Xbox Controller", defaultXbox);
        controllerChooser.addOption("Logitech Flight Stick", new LogitechFlightStick(1));
        controllerChooser.addOption("Simulation Keyboard", new SimulationKeyboard(2));
        controllerChooser.addOption("8BitDo Controller", new Xbox(3));
        controllerChooser.addOption("Linux 8BitDo Controller", new Linux8BitDo(4));
        DashboardManager.setupControllerChooser(controllerChooser);
    }

    /**
     * Returns the Xbox controller if connected to the real field, otherwise returns whatever is selected on the
     * Controller Chooser in Elastic.
     */
    private Controller getActiveController() {
        return DriverStation.isFMSAttached() ? defaultXbox : controllerChooser.getSelected();
    }

    @Override
    Translation2d getTranslation() {
        return getActiveController().getTranslation();
    }

    @Override
    double getRotation() {
        return getActiveController().getRotation();
    }

    @Override
    Trigger runIntake() {
        return new Trigger(() -> getActiveController().runIntake().getAsBoolean());
    }

    @Override
    Trigger shootFuel() {
        return new Trigger(() -> getActiveController().shootFuel().getAsBoolean());
    }

    @Override
    Trigger lowerIntake() {
        return new Trigger(() -> getActiveController().lowerIntake().getAsBoolean());
    }

    @Override
    Trigger raiseIntake() {
        return new Trigger(() -> getActiveController().raiseIntake().getAsBoolean());
    }

    @Override
    Trigger aimHandler() {
        return new Trigger(() -> getActiveController().aimHandler().getAsBoolean());
    }

    @Override
    Trigger manualFlywheel() {
        return new Trigger(() -> getActiveController().manualFlywheel().getAsBoolean());
    }

    @Override
    Trigger seedFieldCentric() {
        return new Trigger(() -> getActiveController().seedFieldCentric().getAsBoolean());
    }

    @Override
    Trigger reverse() {
        return new Trigger(() -> getActiveController().reverse().getAsBoolean());
    }

    @Override
    Trigger trenchPath() {
        return new Trigger(() -> getActiveController().trenchPath().getAsBoolean());
    }

    @Override
    Trigger povUp() {
        return new Trigger(() -> getActiveController().povUp().getAsBoolean());
    }

    @Override
    Trigger povDown() {
        return new Trigger(() -> getActiveController().povDown().getAsBoolean());
    }

    @Override
    Trigger povLeft() {
        return new Trigger(() -> getActiveController().povLeft().getAsBoolean());
    }

    @Override
    Trigger povRight() {
        return new Trigger(() -> getActiveController().povRight().getAsBoolean());
    }

    @Override
    Trigger forwardSysIdQuasi() {
        return new Trigger(() -> getActiveController().forwardSysIdQuasi().getAsBoolean());
    }

    @Override
    Trigger backwardSysIdQuasi() {
        return new Trigger(() -> getActiveController().backwardSysIdQuasi().getAsBoolean());
    }

    @Override
    Trigger forwardSysIdDynamic() {
        return new Trigger(() -> getActiveController().forwardSysIdDynamic().getAsBoolean());
    }

    @Override
    Trigger backwardSysIdDynamic() {
        return new Trigger(() -> getActiveController().backwardSysIdDynamic().getAsBoolean());
    }
}
