package frc.robot.subsystems;

import static frc.robot.utilities.HardwareUtils.*;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DashboardManager;

public class Intake extends SubsystemBase implements SysIdCapable {
    /** Motor that pushes fuel into storage. */
    private final TalonFX pusherMotor = new TalonFX(16);
    final Slot0Configs pusherFFConfig = new Slot0Configs().withKS(0.1).withKV(0.11632).withKA(0.018731);
    /** Motor that extends intake system outside of bumper. */
    private final TalonFX extenderMotor = new TalonFX(17);
    // All units are in rotations
    private final double EXTENDER_STOWED_POSITION = 0.27;
    private final double EXTENDER_DROPPED_POSITION = 0;
    private final double TOLERANCE = 0.07;
    /** Motor that is close to the floor and scoops fuel into pusher. */
    private final TalonFX scooperMotor = new TalonFX(18);
    final Slot0Configs scooperFFConfig = new Slot0Configs().withKS(0.025896).withKV(0.4199).withKA(0.0096081);

    // --PUSHER SYSID CONSTANTS--
    // kS: -0.12147 ERRONEOUS
    // kV: 0.11632
    // kA: 0.018731

    // --SCOOPER SYSID CONSTANTS--
    // kS: 0.15186
    // kV: 2.5262
    // kA: 0.11205

    // --- SysId Configuration (Rollers Only) ---
    // ONLY UNCOMMENT THE ROUTINE CREATION LINE FOR THE MOTOR YOU WANT TO CHARACTERIZE.
    // MAKE SURE TO COMMENT OUT THE OTHER ONE BEFORE RUNNING ANY CHARACTERIZATION COMMANDS.

    // --PUSHER MOTOR--
    private final SysIdRoutine routine = SysIdCapable.createAngularRoutine(this, pusherMotor, pusherMotor::setVoltage);

    // --SCOOPER MOTOR--
    // private final SysIdRoutine routine =
    //     SysIdCapable.createAngularRoutine(this, scooperMotor, scooperMotor::setVoltage);

    public Intake() {
        setMotorCounterClockwisePositive(pusherMotor);
        setMotorCounterClockwisePositive(scooperMotor);
        applyGearRatio(4, scooperMotor);
        applyGearRatio(1, pusherMotor);
        applyGearRatio(36, extenderMotor);
        applyFeedforward(pusherFFConfig, pusherMotor);
        applyFeedforward(scooperFFConfig, scooperMotor);
        extenderMotor.setPosition(EXTENDER_STOWED_POSITION);
        extenderMotor.setNeutralMode(NeutralModeValue.Brake);
        DashboardManager.setupIntake(this::getExtenderPosition, () -> scooperMotor.getVelocity().getValueAsDouble(),
            this::isScooperSpinning);
        DashboardManager.putScooperReverseTimers();
    }

    @Override
    public void periodic() {}

    /** @return The SysIdRoutine used to generate characterization commands. */
    public SysIdRoutine getSysIdRoutine() {
        return routine;
    }

    public Command spinIntakeMotors() {
        double scooperMotorVelocity = DashboardManager.getScooperVelocity();
        double pusherMotorVelocity = DashboardManager.getPusherVelocity();

        return startEnd(() -> {
            setVelocity(scooperMotorVelocity, scooperMotor);
            setVelocity(pusherMotorVelocity, pusherMotor);
        }, () -> {
            stopPusher();
            stopScooper();
        });
    }

    public Command spinIntakeMotorsVoltage() {
        double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
        double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();

        return startEnd(() -> {
            scooperMotor.setVoltage(scooperMotorVoltage);
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> {
            stopPusher();
            stopScooper();
        });
    }

    public Command spinIntakeMotorsVoltageAutoReverse() {
        Timer autoReverseTimer = new Timer();
        double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
        double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
        double loopTime = DashboardManager.getScooperReverseLoopTime();
        double reverseDelay = DashboardManager.getScooperReverseDelay();

        return new FunctionalCommand(
            // initialize
            () -> {
                autoReverseTimer.restart();
                scooperMotor.setVoltage(scooperMotorVoltage);
                pusherMotor.setVoltage(pusherMotorVoltage);
            },
            // execute
            () -> {
                if (autoReverseTimer.get() > loopTime) {
                    if (!isScooperSpinning() && autoReverseTimer.get() > reverseDelay) {
                        autoReverseTimer.restart();
                        scooperMotor.setVoltage(12);
                    } else {
                        scooperMotor.setVoltage(scooperMotorVoltage);
                    }
                }
            },
            // end
            interrupted -> {
                stopPusher();
                stopScooper();
            },

            // isFinished
            () -> false, this);
    }

    public Command testScooper() {
        double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();

        return startEnd(() -> {
            scooperMotor.setVoltage(scooperMotorVoltage);
        }, this::stopScooper);
    }

    public Command reverseScooper() {
        return startEnd(() -> scooperMotor.setVoltage(-12), this::stopScooper);
    }

    public boolean isScooperSpinning() {
        return Math.abs(scooperMotor.getVelocity().getValueAsDouble()) > DashboardManager.getScooperMinSpeed();
    }

    public Command testPusher() {
        double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();

        return startEnd(() -> {
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, this::stopPusher);
    }

    public Command reversePusher() {
        return startEnd(() -> pusherMotor.setVoltage(-12), this::stopPusher);
    }

    public Command testExtender() {
        double extenderMotorVoltage = DashboardManager.getExtenderMotorTestVoltage();

        return startEnd(() -> {
            extenderMotor.setVoltage(extenderMotorVoltage);
        }, this::stopExtender);
    }

    public void spinPusher() {
        double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
        pusherMotor.setVoltage(pusherMotorVoltage);
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

    public Command bumpExtenderUp() {
        return startEnd(() -> extenderMotor.setVoltage(2), this::stopExtender);
    }

    public Command bumpExtenderDown() {
        return startEnd(() -> extenderMotor.setVoltage(-1), this::stopExtender);
    }

    /** This command does not require the intake subsystem to be free and does not lock it when used. */
    public Command bumpExtenderDownNoLockAuto() {
        return Commands.startEnd(() -> extenderMotor.setVoltage(-3), this::stopExtender);
    }

    /** Used for teleop, not permanent. */
    public Command bumpExtenderDownNoLock() {
        return Commands.startEnd(() -> extenderMotor.setVoltage(-1), this::stopExtender);
    }

    /** Used for teleop, not permanent. */
    public Command bumpExtenderUpNoLock() {
        return Commands.startEnd(() -> extenderMotor.setVoltage(2), this::stopExtender);
    }

    /**
     * This is only meant to be used for autonomous paths, in conjunction with the Run Intake Named Command, to keep the
     * intake from popping up when collecting fuel.
     */
    public Command keepExtenderDownNoLock() {
        return Commands.startEnd(() -> extenderMotor.setVoltage(-0.25), this::stopExtender);
    }

    /** Used to avoid the subsystem locking for this command. */
    public void setExtenderVoltagePositive() {
        extenderMotor.setVoltage(2);
    }

    /** Used to avoid the subsystem locking for this command. */
    public void setExtenderVoltageNegative() {
        extenderMotor.setVoltage(-2);
    }

    /** @return Extender motor's position in REVOLUTIONS */
    public double getExtenderPosition() {
        return extenderMotor.getPosition().getValueAsDouble();
    }

    public Command dropArm() {
        return new FunctionalCommand(
            // --initialize--
            () -> extenderMotor.setVoltage(-0.7),

            // --execute--
            () -> {},

            // --end--
            interrupted -> stopExtender(),

            // --isFinished--
            () -> getExtenderPosition() <= EXTENDER_DROPPED_POSITION + TOLERANCE,

            // --addRequirements--
            this); // Pass in Intake
    }

    public Command raiseArm() {
        return new FunctionalCommand(
            // --initialize--
            () -> extenderMotor.setVoltage(1),

            // --execute--
            () -> {},

            // --end--
            interrupted -> stopExtender(),

            // --isFinished--
            () -> getExtenderPosition() >= EXTENDER_STOWED_POSITION * 0.75 - TOLERANCE,

            // --addRequirements--
            this); // Pass in Intake
    }

    public Command zeroExtenderPosition() {
        return runOnce(() -> extenderMotor.setPosition(0));
    }
}
