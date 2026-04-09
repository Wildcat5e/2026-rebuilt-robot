package frc.robot.subsystems;

import static frc.robot.utilities.HardwareUtils.*;
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
    private final double PUSHER_RADIUS = 0.058 / 2;
    private final double PUSHER_CIRCUMFERENCE = 2 * Math.PI * PUSHER_RADIUS;
    /** Motor that extends intake system outside of bumper. */
    private final TalonFX extenderMotor = new TalonFX(17);
    // All units are in rotations
    private double EXTENDER_STOWED_POSITION = .27;
    private double EXTENDER_DROPPED_POSITION = 0;
    private double TOLERANCE = 0.07;
    /** Motor that is close to the floor and scoops fuel into pusher. */
    private final TalonFX scooperMotor = new TalonFX(18);
    private final double SCOOPER_RADIUS = 0.06858 / 2;
    private final double SCOOPER_CIRCUMFERENCE = 2 * Math.PI * SCOOPER_RADIUS;
    private final Timer autoReverseTimer = new Timer();

    // --- SysId Configuration (Rollers Only) ---
    // ONLY UNCOMMENT THE ROUTINE CREATION LINE FOR THE MOTOR YOU WANT TO CHARACTERIZE.
    // MAKE SURE TO COMMENT OUT THE OTHER ONE BEFORE RUNNING ANY CHARACTERIZATION COMMANDS.

    // --PUSHER MOTOR--
    private final SysIdRoutine routine = SysIdCapable.createAngularRoutine(this, pusherMotor, pusherMotor::setVoltage);

    // --SCOOPER MOTOR--
    // private final SysIdRoutine routine =
    //     SysIdCapable.createLinearRoutine(this, scooperMotor, scooperMotor::setVoltage, SCOOPER_CIRCUMFERENCE);

    public Intake() {
        applyGearRatio(1, scooperMotor, pusherMotor);
        applyGearRatio(36, extenderMotor);
        extenderMotor.setPosition(EXTENDER_STOWED_POSITION);
        extenderMotor.setNeutralMode(NeutralModeValue.Brake);
        DashboardManager.setupIntake(this::getExtenderPosition, this::isScooperSpinning);
        DashboardManager.putScooperReverseTimers();
    }

    @Override
    public void periodic() {}

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
            stopPusher();
            stopScooper();
        });
    }

    public Command spinIntakeMotorsAutoReverse() {
        return new FunctionalCommand(
            // initialize
            () -> {
                autoReverseTimer.restart();
                double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
                double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
                scooperMotor.setVoltage(-scooperMotorVoltage);
                pusherMotor.setVoltage(pusherMotorVoltage);
            },
            // execute
            () -> {
                double loopTime = DashboardManager.getScooperReverseLoopTime();
                double reverseDelay = DashboardManager.getScooperReverseDelay();
                if (autoReverseTimer.get() > loopTime) {
                    if (!isScooperSpinning() && autoReverseTimer.get() > reverseDelay) {
                        autoReverseTimer.restart();
                        scooperMotor.setVoltage(12);
                    } else {
                        double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
                        scooperMotor.setVoltage(-scooperMotorVoltage);
                    }
                }
            },
            // end
            interrupted -> {
                stopPusher();
                stopScooper();
            }, () -> false, this);
    }

    public Command testScooper() {
        return startEnd(() -> {
            double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
            scooperMotor.setVoltage(-scooperMotorVoltage);
        }, this::stopScooper);
    }

    public Command reverseScooper() {
        return startEnd(() -> scooperMotor.setVoltage(12), this::stopScooper).withName("Reverse Scooper");
    }

    public boolean isScooperSpinning() {
        return Math.abs(scooperMotor.getVelocity().getValueAsDouble()) > DashboardManager.getScooperMinSpeed();
    }

    public Command testPusher() {
        return startEnd(() -> {
            double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, this::stopPusher);
    }

    public Command reversePusher() {
        return startEnd(() -> pusherMotor.setVoltage(-12), this::stopPusher).withName("Reverse Pusher");
    }

    public Command testExtender() {
        return startEnd(() -> {
            double extenderMotorVoltage = DashboardManager.getExtenderMotorTestVoltage();
            extenderMotor.setVoltage(extenderMotorVoltage);
        }, this::stopExtender);
    }

    public Command dropIntake() {
        return startEnd(() -> extenderMotor.setVoltage(-1), this::stopExtender);
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
        return startEnd(() -> extenderMotor.setVoltage(2), this::stopExtender).withName("Bump Extender Up");
    }

    public Command bumpExtenderDown() {
        return startEnd(() -> extenderMotor.setVoltage(-1), this::stopExtender).withName("Bump Extender Down");
    }

    /**
     * This command is functionally the same as bumpExtenderDown(), but does not require the intake subsystem to be free
     * and does not lock the intake subsystem when used.
     */
    public Command bumpExtenderDownNoLock() {
        return Commands.startEnd(() -> extenderMotor.setVoltage(-3), this::stopExtender).withName("Bump Extender Down");
    }

    /**
     * This is only meant to be used for autonomous paths, it is meant to be used with the run intake method in order to
     * keep the intake from popping up when collectin fuel
     */
    public Command keepExtenderDownNoLock() {
        return Commands.startEnd(() -> extenderMotor.setVoltage(-0.25), this::stopExtender);
    }

    // Used to avoid the subsystem locking when scheduling a command
    public void setExtenderVoltagePositive() {
        extenderMotor.setVoltage(1);
    }

    public void setExtenderVoltageNegative() {
        extenderMotor.setVoltage(-1);
    }

    /** @return Extender motor's position in REVOLUTIONS */
    public double getExtenderPosition() {
        return extenderMotor.getPosition().getValueAsDouble();
    }

    public Command dropArmFinalImplementation() {
        return new FunctionalCommand(
            // --initialize--
            () -> extenderMotor.setVoltage(-0.7),

            // --execute--
            () -> {},

            // --end--
            interrupted -> extenderMotor.setVoltage(0),

            // --isFinished--
            () -> {
                return getExtenderPosition() <= EXTENDER_DROPPED_POSITION + TOLERANCE; // Check sign
            },
            // --addRequirements--
            this); // Pass in Intake
    }

    public Command raiseArmFinalImplementation() {
        return new FunctionalCommand(
            // --initialize--
            () -> extenderMotor.setVoltage(1),

            // --execute--
            () -> {},

            // --end--
            interrupted -> extenderMotor.setVoltage(0),

            // --isFinished--
            () -> {
                return getExtenderPosition() >= EXTENDER_STOWED_POSITION * 0.75 - TOLERANCE; // Check sign
            },
            // --addRequirements--
            this); // Pass in Intake
    }

    public Command zeroExtenderPosition() {
        return runOnce(() -> extenderMotor.setPosition(0));
    }
}
