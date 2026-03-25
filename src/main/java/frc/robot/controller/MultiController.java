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
public class MultiController implements Controller {
    private final SendableChooser<Controller> controllerChooser = new SendableChooser<>();

    public MultiController() {
        DriverStation.silenceJoystickConnectionWarning(true);
        controllerChooser.setDefaultOption("Xbox Controller", new Xbox(0));
        if (DriverStation.isTest()) {
            controllerChooser.addOption("Logitech Flight Stick", new LogitechFlightStick(1));
            controllerChooser.addOption("Simulation Keyboard", new SimulationKeyboard(2));
            controllerChooser.addOption("8BitDo Controller", new Xbox(3));
            controllerChooser.addOption("Linux 8BitDo Controller", new Linux8BitDo(4));
        }
        SmartDashboard.putData("Controller Chooser", controllerChooser);
    }

    private Controller controller() {return controllerChooser.getSelected();}

    @Override public double rotation() {return controller().rotation();}

    @Override public Translation2d translation() {return controller().translation();}

    @Override public Trigger runIntake() {return controller().runIntake();}

    @Override public Trigger shootFuel() {return controller().shootFuel();}

    @Override public Trigger lowerIntake() {return controller().lowerIntake();}

    @Override public Trigger raiseIntake() {return controller().raiseIntake();}

    @Override public Trigger aimHandler() {return controller().aimHandler();}

    @Override public Trigger manualFlywheel() {return controller().manualFlywheel();}

    @Override public Trigger seedFieldCentric() {return controller().seedFieldCentric();}

    @Override public Trigger reverse() {return controller().reverse();}

    @Override public Trigger povUp() {return controller().povUp();}

    @Override public Trigger povDown() {return controller().povDown();}

    @Override public Trigger povLeft() {return controller().povLeft();}

    @Override public Trigger povRight() {return controller().povRight();}

    @Override public Trigger forwardSysIdQuasi() {return controller().forwardSysIdQuasi();}

    @Override public Trigger backwardSysIdQuasi() {return controller().backwardSysIdQuasi();}

    @Override public Trigger forwardSysIdDynamic() {return controller().forwardSysIdDynamic();}

    @Override public Trigger backwardSysIdDynamic() {return controller().backwardSysIdDynamic();}
}
