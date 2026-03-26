package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.ShootFuel;
import frc.robot.controller.Controller;
import frc.robot.controller.MultiController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

import static com.pathplanner.lib.auto.NamedCommands.registerCommand;
import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.DashboardManager.incrementFlywheelSpeedMultiplier;
import static frc.robot.DashboardManager.incrementStaticFlywheelSpeed;
import static frc.robot.Utilities.getHubPosition;

public class Robot extends TimedRobot {
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Flywheel flywheel = new Flywheel(drivetrain);
    public final Hopper hopper = new Hopper();
    public final Intake intake = new Intake();
    public final PhotonVision vision = new PhotonVision(drivetrain::addVisionMeasurement);

    private final Controller controller = new MultiController();
    private final CommandGenericHID macropad = new CommandGenericHID(4);
    private final Field2d fieldWidget = new Field2d();

    // https://github.com/Gold872/elastic_dashboard/blob/v2026.1.1/elasticlib/Elastic.java

    private final StringPublisher elasticTabPublisher = NetworkTableInstance.getDefault()
        .getStringTopic("/Elastic/SelectedTab").publish(PubSubOption.keepDuplicates(true));

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    private Simulation simulation;
    public boolean manualRotation = true;

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    public Robot() {
        configureAutoBuilder();
        bindController();
        bindMacropad();

        registerCommand("Drop Intake", intake.bumpExtenderDown());
        registerCommand("Run Intake", intake.spinIntakeMotors());
        registerCommand("Aim at Hub", AimAtTarget.hub(this));
        registerCommand("Shoot Fuel", new ShootFuel(flywheel, hopper, () -> Utilities.inHome(drivetrain)));

        SignalLogger.enableAutoLogging(false);

        DashboardManager.setupRobotInit(fieldWidget, autoChooser, drivetrain);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        fieldWidget.setRobotPose(drivetrain.getState().Pose);
        DashboardManager.updateRobotPeriodic(drivetrain, getHubPosition());
    }

    @Override
    public void autonomousInit() {
        elasticTabPublisher.set("Autonomous");
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().schedule(autoChooser.getSelected());
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        elasticTabPublisher.set("Teleoperated");
        if (autoChooser.getSelected() != null) {
            CommandScheduler.getInstance().cancel(autoChooser.getSelected());
        }
    }

    @Override
    public void simulationInit() {
        simulation = new Simulation(drivetrain);
    }

    @Override
    public void simulationPeriodic() {
        simulation.poseUpdate();
    }

    /**
     * This configures {@link AutoBuilder} and must be run before creating commands that use it.
     */
    public void configureAutoBuilder() {// @formatter:off
        try {
            var applyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,   // Supplier of current robot pose
                drivetrain::resetPose,              // Consumer for seeding pose against auto
                () -> drivetrain.getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> drivetrain.setControl(
                    applyRobotSpeedsRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }// @formatter:on

    private static final double MAX_ANGULAR_SPEED = Constants.MAX_ANGULAR_SPEED - Math.PI;
    private static final double RADIAL_DEAD_ZONE = .15;
    private static final double SCALE_EXPONENT = 1;

    public void bindMacropad() {
        // LAYER 0 (No Modifiers)
        macropad.button(1).whileTrue(intake.bumpExtenderUp());
        macropad.button(2).whileTrue(intake.bumpExtenderDown());
        macropad.button(3).onTrue(runOnce(() -> incrementFlywheelSpeedMultiplier(0.01)).ignoringDisable(true));
        macropad.button(4).onTrue(runOnce(() -> incrementFlywheelSpeedMultiplier(-0.01)).ignoringDisable(true));
        macropad.button(15).onTrue(runOnce(() -> incrementStaticFlywheelSpeed(0.5)).ignoringDisable(true));
        macropad.button(16).onTrue(runOnce(() -> incrementStaticFlywheelSpeed(-0.5)).ignoringDisable(true));
        macropad.button(17).whileTrue(flywheel.tunableFlywheelSpeedCommand());
        macropad.button(18).whileTrue(hopper.testTunableKicker());

        // LAYER 1 (Shift Held)
        macropad.button(5).whileTrue(intake.reverseScooper());
        macropad.button(6).whileTrue(intake.reversePusher());
        macropad.button(7).whileTrue(hopper.reverseConveyor());
        macropad.button(8).whileTrue(hopper.reverseKicker());
        macropad.button(9).whileTrue(flywheel.reverseFlywheel());

        // Emergency Stop
        if (DriverStation.isTest()) {
            macropad.button(10)
                .whileTrue(Commands.run(() -> CommandScheduler.getInstance().cancelAll()).ignoringDisable(true));
        }

        // LAYER 2 (Control Held)
        macropad.button(11).whileTrue(hopper.runHopper());
        macropad.button(12).whileTrue(flywheel.backupFlywheelL1());
        macropad.button(13).whileTrue(flywheel.backupFlywheelL2());
        macropad.button(14).whileTrue(flywheel.backupFlywheelL3());
    }

    private void bindController() {
        controller.shootFuel().whileTrue(new ShootFuel(flywheel, hopper, () -> Utilities.inHome(drivetrain)));
        controller.runIntake().whileTrue(intake.spinIntakeMotors());
        controller.lowerIntake().whileTrue(intake.testExtender());
        controller.aimHandler().whileTrue(AimAtTarget.hub(this));
        controller.manualFlywheel().whileTrue(flywheel.tunableFlywheelVoltageCommand());
        controller.seedFieldCentric().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        controller.reverse().whileTrue(intake.reverseScooper());
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            double x = controller.translation().getX();
            double y = controller.translation().getY();
            double magnitude = Math.hypot(x, y);

            Translation2d translation;
            if (magnitude < RADIAL_DEAD_ZONE) {
                translation = new Translation2d(0, 0);
            } else {
                translation = new Translation2d(x, y).div(magnitude)
                    .times(Math.pow(applyDeadband(magnitude, RADIAL_DEAD_ZONE), SCALE_EXPONENT));
            }
            SwerveRequest.FieldCentric fieldCentric =
                new SwerveRequest.FieldCentric().withVelocityX(translation.getX() * Constants.MAX_LINEAR_SPEED)
                    .withVelocityY(translation.getY() * Constants.MAX_LINEAR_SPEED);

            if (manualRotation) {
                fieldCentric.withRotationalRate(controller.rotation() * MAX_ANGULAR_SPEED);
            }

            return fieldCentric;
        }));
    }
}
