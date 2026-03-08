package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands;
import frc.robot.Constants;

/**
 * Controller management.
 * 
 * @apiNote Each axis uses the
 *          <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">WPILib
 *          coordinate system<a>.
 */
public abstract class Controller {
    /** Deadzone to apply to joysticks as a proportion out of 1. */
    private static final double DEADZONE = .15;
    /** Exponent to raise inputs to the power of to create a curved response. */
    private static final double SCALE_EXPONENT = 1;
    /** Limit max controller angular speed to prevent flicking robot around too fast and spilling balls. */
    private static final double MAX_ANGULAR_SPEED = Constants.MAX_ANGULAR_SPEED - Math.PI; // Check if this is even needed or reasonable.
    /** Use this to create requests for driving the robot and use the drivetrain to apply them. */
    private static final SwerveRequest.FieldCentric swerveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    /** Change whether or not controller can control translation. */
    public static boolean allowControllerTranslation = true;
    /** Change whether or not controller can control rotation. */
    public static boolean allowControllerRotation = true;

    /** Get Translation2d of controller axes. */
    public abstract Translation2d getTranslation();

    /** Get the rotation axis value. @return The axis value. */
    public abstract double getRotation();

    /** Xbox left trigger to activate the intake. */
    public abstract Trigger activateIntake();

    /** Xbox right trigger to shoot fuel. Flight Stick trigger. */
    public abstract Trigger shootFuel();

    /** Xbox button A to rotate to hub. */
    public abstract Trigger rotateToHub();

    /** Xbox right bumper to lower the intake. */
    public abstract Trigger lowerIntake();

    /** Xbox left bumper to raise the intake. */
    public abstract Trigger raiseIntake();

    /** Xbox buttons Start and Y to forwardSysIdQuasi. */
    public abstract Trigger forwardSysIdQuasi();

    /** Xbox buttons Start and X to backwardSysIdQuasi. */
    public abstract Trigger backwardSysIdQuasi();

    /** Xbox buttons Back and Y to forwardSysIdDynamic. */
    public abstract Trigger forwardSysIdDynamic();

    /** Xbox buttons Back and X to backwardSysIdDynamic. */
    public abstract Trigger backwardSysIdDynamic();


    /** Sets up key/button/joystick bindings for driving and controlling the robot. */
    public void bindingsSetup(Drivetrain drivetrain, Commands commands) {
        // reset the field-centric heading on left trigger
        activateIntake().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // a and right bumper
        // rotateToHub().whileTrue(new RotateToHub(drivetrain, true)); // PID + Shooting Calculator testing
        // lowerIntake().whileTrue(new RotateToHub(drivetrain, false)); // Pure Feedforward + PID testing

        // shootFuel().whileTrue(commands.flywheel.testTunableFlywheel()); // b, change to right trigger
        // shootFuel().whileTrue(commands.hopper.testTunableKicker()); // b, change to right trigger

        Controller.joystick.povUp().whileTrue(commands.flywheel.sysIdDynamicForward());
        Controller.joystick.povRight().whileTrue(commands.flywheel.sysIdDynamicReverse());
        Controller.joystick.povDown().whileTrue(commands.flywheel.sysIdQuasistaticForward());
        Controller.joystick.povLeft().whileTrue(commands.flywheel.sysIdQuasistaticReverse());
        /*
         * Tests for motor identification:
         * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
         * https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration
         */
        // Quasistatic test for motor identification
        forwardSysIdQuasi().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        backwardSysIdQuasi().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // Dynamic test for motor identification
        forwardSysIdDynamic().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        backwardSysIdDynamic().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            var translation = getTranslation();
            if (allowControllerTranslation) {
                swerveRequest.withVelocityX(translation.getX() * Constants.MAX_LINEAR_SPEED)
                    .withVelocityY(translation.getY() * Constants.MAX_LINEAR_SPEED);
            }
            if (allowControllerRotation) {
                swerveRequest.withRotationalRate(getRotation() * MAX_ANGULAR_SPEED);
            }
            return swerveRequest;
        }));
    }

    public static class Xbox extends Controller {
        private final CommandXboxController controller;

        /** Uses {@link CommandXboxController}. @param port index on Driver Station */
        public Xbox(int port) {
            controller = new CommandXboxController(port);
        }

        @Override
        public Translation2d getTranslation() {
            return applyRadialDeadzone(-controller.getLeftY(), -controller.getLeftX(), DEADZONE);
        }

        @Override
        public double getRotation() {
            return MathUtil.applyDeadband(-controller.getRightX(), DEADZONE);
        }

        @Override
        public Trigger activateIntake() {
            return controller.leftTrigger();
        }

        @Override
        public Trigger shootFuel() {
            return controller.b(); // change to right trigger
        }

        @Override
        public Trigger rotateToHub() {
            return controller.a();
        }

        @Override
        public Trigger lowerIntake() {
            return controller.rightBumper();
        }

        @Override
        public Trigger raiseIntake() {

            return controller.leftBumper();
        }

        @Override
        public Trigger forwardSysIdQuasi() {
            return controller.start().and(controller.y());
        }

        @Override
        public Trigger backwardSysIdQuasi() {
            return controller.start().and(controller.x());
        }

        @Override
        public Trigger forwardSysIdDynamic() {
            return controller.back().and(controller.y());
        }

        @Override
        public Trigger backwardSysIdDynamic() {
            return controller.back().and(controller.x());
        }
    }
    public static class LogitechFlightStick extends Controller {
        private final CommandJoystick controller;
        /** Deadzone specific to flight stick. */
        // private static final double DEADZONE = ControllerWrapper.DEADZONE; // for now use main deadzone

        /** Uses {@link CommandJoystick} for Logitech Extreme 3D Pro. @param port index on Driver Station */
        public LogitechFlightStick(int port) {
            controller = new CommandJoystick(port);
        }

        @Override
        public Translation2d getTranslation() {
            return applyRadialDeadzone(-controller.getRawAxis(1), -controller.getRawAxis(0), DEADZONE);
        }

        @Override
        public double getRotation() {
            return MathUtil.applyDeadband(-controller.getRawAxis(2), .3);
        }

        @Override
        public Trigger activateIntake() {
            /* change this */ return controller.button(0);
        }

        @Override
        public Trigger shootFuel() {
            return controller.trigger();
        }

        @Override
        public Trigger rotateToHub() {
            /* change this */ return controller.button(1);
        }

        @Override
        public Trigger lowerIntake() {
            /* change this */ return controller.button(2);
        }

        @Override
        public Trigger raiseIntake() {
            /* change this */ return controller.button(0);
        }

        @Override
        public Trigger forwardSysIdQuasi() {
            /* change this */ return controller.button(0);
            // return Controller.joystick.start().and(Controller.joystick.y());
        }

        @Override
        public Trigger backwardSysIdQuasi() {
            /* change this */ return controller.button(0);
            // return Controller.joystick.start().and(Controller.joystick.x());
        }

        @Override
        public Trigger forwardSysIdDynamic() {
            /* change this */ return controller.button(0);
            // return Controller.joystick.back().and(Controller.joystick.y());
        }

        @Override
        public Trigger backwardSysIdDynamic() {
            /* change this */ return controller.button(0);
            // return Controller.joystick.back().and(Controller.joystick.x());
        }
    }
    public static class SimulationKeyboard extends LogitechFlightStick {
        private final CommandJoystick controller;

        /** Uses {@link CommandJoystick} for Simulation Keyboard. @param port index on Driver Station */
        public SimulationKeyboard(int port) {
            super(port);
            controller = super.controller;
        }
    }
    /**
     * Controller that will use the {@link Controller} selected by the dashboard widget. For example, you can select the
     * {@link Xbox} controller. Also, this silences unplugged controller warnings. You should probably directly us your
     * controller before competition instead of using this.
     */
    public static class MultiController extends Controller {
        final SendableChooser<Controller> controllerChooser = new SendableChooser<Controller>();

        public MultiController(Drivetrain drivetrain, Commands commands) {
            DriverStation.silenceJoystickConnectionWarning(true);
            controllerChooser.setDefaultOption("Xbox Controller", new Controller.Xbox(0));
            controllerChooser.addOption("Logitech Flight Stick", new Controller.LogitechFlightStick(1));
            controllerChooser.addOption("Simulation Keyboard", new Controller.SimulationKeyboard(2));
            SmartDashboard.putData("Controller Chooser", controllerChooser);
            controllerChooser.onChange(controller -> bindingsSetup(drivetrain, commands));
            // Extra unneeded call just for good measure in case the sim GUI updates the chooser but Elastic with the actual robot doesn't.
            bindingsSetup(drivetrain, commands);
        }

        @Override
        public double getRotation() {
            return controllerChooser.getSelected().getRotation();
        }

        @Override
        public Translation2d getTranslation() {
            return controllerChooser.getSelected().getTranslation();
        }

        @Override
        public Trigger activateIntake() {
            return controllerChooser.getSelected().activateIntake();
        }

        @Override
        public Trigger shootFuel() {
            return controllerChooser.getSelected().shootFuel();
        }

        @Override
        public Trigger rotateToHub() {
            return controllerChooser.getSelected().rotateToHub();
        }

        @Override
        public Trigger lowerIntake() {
            return controllerChooser.getSelected().lowerIntake();
        }

        @Override
        public Trigger raiseIntake() {
            return controllerChooser.getSelected().raiseIntake();
        }

        @Override
        public Trigger forwardSysIdQuasi() {
            return controllerChooser.getSelected().forwardSysIdQuasi();
        }

        @Override
        public Trigger backwardSysIdQuasi() {
            return controllerChooser.getSelected().backwardSysIdQuasi();
        }

        @Override
        public Trigger forwardSysIdDynamic() {
            return controllerChooser.getSelected().forwardSysIdDynamic();
        }

        @Override
        public Trigger backwardSysIdDynamic() {
            return controllerChooser.getSelected().backwardSysIdDynamic();
        }
    }

    /**
     * APPLY FIRST! Applies a deadzone as a proportion of the input. Values shifted up out of deadzone and compressed
     * outside deadzone. The max value of 1 remains at the max. This is a scaled radial deadzone. Also, curves input.
     * 
     * @param xAxis raw value from controller
     * @param yAxis raw value from controller
     * @param deadzone proportion to eliminate
     * @return axis values in Translation2d
     */
    static Translation2d applyRadialDeadzone(double xAxis, double yAxis, double deadzone) {
        double magnitude = Math.hypot(xAxis, yAxis);
        if (magnitude < deadzone) {
            return new Translation2d(0, 0);
        }
        double scaledMagnitude = Math.pow(MathUtil.applyDeadband(magnitude, deadzone), SCALE_EXPONENT);
        return new Translation2d(xAxis, yAxis).div(magnitude).times(scaledMagnitude);
    }
}
