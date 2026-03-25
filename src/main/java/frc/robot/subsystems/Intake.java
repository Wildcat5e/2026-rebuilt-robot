package frc.robot.subsystems;

import static frc.robot.utilities.HardwareUtils.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DashboardManager;
import static frc.robot.subsystems.SysIdCapable.*;

public class Intake extends SubsystemBase implements SysIdCapable, MotionMagicCapable {


    /** Motor that shoots fuel into the robot */
    private final TalonFX pusherMotor = new TalonFX(16);
    private final double PUSHER_RADIUS = 0.058 / 2;
    private final double PUSHER_CIRCUMFERENCE = 2 * Math.PI * PUSHER_RADIUS;

    /** Motor that extends intake system outside of bumper. */
    private final TalonFX extenderMotor = new TalonFX(17);
    private final MotionMagicVoltage extenderMotionRequest = new MotionMagicVoltage(0);
    /* Measured in mechanism revolutions. Depends on SensorToMechanismRatio. */
    private final double EXTENDER_STOWED_POSITION = 120.0 / 360.0; // Roughly 120 degrees from the horizontal
    /* Measured in mechanism revolutions. Depends on SensorToMechanismRatio. */
    private final double EXTENDER_DROPPED_POSITION = 0.0;
    /*
     * Measured in mechanism revolutions. Represents how close to the target position the extender must be before the
     * moveToPosition command stops. Depends on SensorToMechanismRatio.
     */
    private final double EXTENDER_TOLERANCE = 0.02; // In rotations of the motor (not the output)

    /** Motor that is closer to the floor and scoops fuel into pusher. */
    private final TalonFX scooperMotor = new TalonFX(18);
    private final double SCOOPER_RADIUS = 0.06858 / 2;
    private final double SCOOPER_CIRCUMFERENCE = 2 * Math.PI * SCOOPER_RADIUS;

    // --- SysId Configuration (Rollers Only) ---
    // ONLY UNCOMMENT THE ROUTINE CREATION LINE FOR THE MOTOR YOU WANT TO CHARACTERIZE.
    // MAKE SURE TO COMMENT OUT THE OTHER ONE BEFORE RUNNING ANY CHARACTERIZATION COMMANDS.

    // --PUSHER MOTOR--
    private final SysIdRoutine routine =
        createLinearRoutine(this, pusherMotor, pusherMotor::setVoltage, PUSHER_CIRCUMFERENCE);

    // --SCOOPER MOTOR--
    // private final SysIdRoutine routine =
    //     createLinearRoutine(this, scooperMotor, scooperMotor::setVoltage, SCOOPER_CIRCUMFERENCE);

    public Intake() {
        applyGearRatio(pusherMotor, 1);
        applyGearRatio(extenderMotor, 36);
        applyGearRatio(scooperMotor, 1);

        // --- Extender Motion Magic Setup ---
        // Assume arm starts stowed at the beginning of the match
        extenderMotor.setPosition(EXTENDER_STOWED_POSITION);
        // Initializing with zeros; actual values will be pushed via configureMotor()
        MotionMagicCapable.configureMotionMagic(extenderMotor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.33, 0.66, 0.0);

        DashboardManager.setupIntake(this::getMechanismPosition, this::isScooperSpinning);
    }

    @Override
    public void periodic() {}

    // --- MotionMagicCapable Interface Requirements ---
    @Override
    public TalonFX getMotionMagicMotor() {
        return extenderMotor;
    }

    @Override
    public MotionMagicVoltage getMotionMagicRequest() {
        return extenderMotionRequest;
    }

    @Override
    public Subsystem getAssociatedSubsystem() {
        return this;
    }

    /** @return The SysIdRoutine used to generate characterization commands. */
    public SysIdRoutine getSysIdRoutine() {
        return routine;
    }

    public Command spinIntakeMotors() {
        return startEnd(() -> {
            double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
            double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
            scooperMotor.setVoltage(-scooperMotorVoltage);
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> {
            scooperMotor.setVoltage(3);
            stopPusher();
        });
    }

    public void spinPusher() {
        double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
        pusherMotor.setVoltage(pusherMotorVoltage);
    }

    public Command reversePusher() {
        return startEnd(() -> pusherMotor.setVoltage(3), () -> stopPusher()).withName("Reverse Pusher");
    }

    public void stopPusher() {
        pusherMotor.setVoltage(0);
    }

    public void stopExtender() {
        extenderMotor.setVoltage(0);
    }

    public void stopScooper() {
        scooperMotor.setVoltage(0);
    }

    public Command testPusher() {
        return startEnd(() -> {
            double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> pusherMotor.set(0));
    }

    public Command configureExtenderMotor() {
        return runOnce(() -> {
            MotionMagicCapable.configureMotionMagic(extenderMotor, DashboardManager.getExtenderkP(),
                DashboardManager.getExtenderkI(), DashboardManager.getExtenderkD(), DashboardManager.getExtenderkS(),
                DashboardManager.getExtenderkV(), DashboardManager.getExtenderkG(),
                DashboardManager.getExtenderMotionMagicCruiseVelocity(),
                DashboardManager.getExtenderMotionMagicAcceleration(), DashboardManager.getExtenderMotionMagicJerk());
        });
    }

    public Command raiseArmFinalImplementation() {
        return moveToPosition(EXTENDER_STOWED_POSITION, EXTENDER_TOLERANCE);
    }

    public Command dropArmFinalImplementation() {
        return moveToPosition(EXTENDER_DROPPED_POSITION, EXTENDER_TOLERANCE);
    }

    public Command bumpExtenderUp() {
        return startEnd(() -> setExtenderStowVoltage(), () -> stopExtender()).withName("Bump Extender Up");
    }

    public Command bumpExtenderDown() {
        return startEnd(() -> setExtenderDeployVoltage(), () -> stopExtender()).withName("Bump Extender Down");
    }

    // Used to avoid the subsystem locking when scheduling a command
    public void setExtenderStowVoltage() {
        extenderMotor.setVoltage(1);
    }

    public void setExtenderDeployVoltage() {
        extenderMotor.setVoltage(-0.5);
    }

    public Command testExtender() {
        return startEnd(() -> {
            double extenderMotorVoltage = DashboardManager.getExtenderMotorTestVoltage();
            extenderMotor.setVoltage(extenderMotorVoltage);
        }, () -> stopExtender());
    }

    public Command testScooper() {
        return startEnd(() -> {
            double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
            scooperMotor.setVoltage(-scooperMotorVoltage);
        }, () -> scooperMotor.set(0));
    }

    public Command reverseScooper() {
        return startEnd(() -> scooperMotor.setVoltage(3), () -> stopScooper()).withName("Reverse Scooper");
    }

    public boolean isScooperSpinning() {
        return Math.abs(scooperMotor.getVelocity().getValueAsDouble()) > 0.1;
    }
}
