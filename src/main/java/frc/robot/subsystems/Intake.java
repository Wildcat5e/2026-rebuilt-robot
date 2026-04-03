package frc.robot.subsystems;

import static frc.robot.utilities.HardwareUtils.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardManager;

public class Intake extends SubsystemBase {
    /** Motor that shoots fuel into the robot */
    private final TalonFX pusherMotor = new TalonFX(16);
    /** Motor that extends intake system outside of bumper. */
    private final TalonFX extenderMotor = new TalonFX(17);
    /** Motor that is closer to the floor and scoops fuel into other set of wheels. */
    private final TalonFX scooperMotor = new TalonFX(18);
    private double extenderMotorPosition = 0;
    private final Timer autoReverseTimer = new Timer();

    public Intake() {
        applyGearRatio(scooperMotor, 1);
        applyGearRatio(pusherMotor, 1);
        applyGearRatio(extenderMotor, 36);
        extenderMotor.setNeutralMode(NeutralModeValue.Brake);
        DashboardManager.setupIntake(() -> extenderMotorPosition, () -> scooperMotor.getVelocity().getValueAsDouble());
        DashboardManager.putScooperReverseTimers();
    }

    @Override
    public void periodic() {
        extenderMotorPosition = getExtenderPosition();
    }

    public Command spinIntakeMotors() {
        return startEnd(() -> {
            double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
            double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
            scooperMotor.setVoltage(-scooperMotorVoltage);
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> {
            scooperMotor.setVoltage(0);
            pusherMotor.setVoltage(0);
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
                double scooperSpeed = Math.abs(scooperMotor.getVelocity().getValueAsDouble());
                if (autoReverseTimer.get() > loopTime) {
                    if (scooperSpeed < 0.1 && autoReverseTimer.get() > reverseDelay) {
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
                scooperMotor.setVoltage(0);
                pusherMotor.setVoltage(0);
            }, () -> false, this);
    }

    public Command testScooper() {
        return startEnd(() -> {
            double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
            scooperMotor.setVoltage(-scooperMotorVoltage);
        }, () -> scooperMotor.set(0));
    }

    public Command reverseScooper() {
        return startEnd(() -> scooperMotor.setVoltage(12), () -> scooperMotor.setVoltage(0))
            .withName("Reverse Scooper");
    }

    public boolean isScooperSpinning() {
        return Math.abs(scooperMotor.getVelocity().getValueAsDouble()) > DashboardManager.getScooperMinSpeed();
    }

    public Command testPusher() {
        return startEnd(() -> {
            double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> pusherMotor.set(0));
    }

    public Command reversePusher() {
        return startEnd(() -> pusherMotor.setVoltage(-12), () -> pusherMotor.setVoltage(0)).withName("Reverse Pusher");
    }

    public Command testExtender() {
        return startEnd(() -> {
            double extenderMotorVoltage = DashboardManager.getExtenderMotorTestVoltage();
            extenderMotor.setVoltage(extenderMotorVoltage);
        }, () -> extenderMotor.setVoltage(0));
    }

    public Command dropIntake() {
        return startEnd(() -> extenderMotor.setVoltage(-1), () -> extenderMotor.setVoltage(0));
    }

    public void spinPusher() {
        double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
        pusherMotor.setVoltage(pusherMotorVoltage);
    }

    public void stopPusher() {
        pusherMotor.setVoltage(0);
    }

    public Command bumpExtenderUp() {
        return startEnd(() -> extenderMotor.setVoltage(2), () -> extenderMotor.setVoltage(0))
            .withName("Bump Extender Up");
    }

    public Command bumpExtenderDown() {
        return startEnd(() -> extenderMotor.setVoltage(-1), () -> extenderMotor.setVoltage(0))
            .withName("Bump Extender Down");
    }

    /**
     * This command is functionally the same as bumpExtenderDown(), but does not require the intake subsystem to be free
     * and does not lock the intake subsystem when used.
     */
    public Command bumpExtenderDownNoLock() {
        return Commands.startEnd(() -> extenderMotor.setVoltage(-3), () -> extenderMotor.setVoltage(0))
            .withName("Bump Extender Down");
    }

    // Used to avoid the subsystem locking when scheduling a command
    public void setExtenderVoltagePositive() {
        extenderMotor.setVoltage(1);
    }

    public void setExtenderVoltageNegative() {
        extenderMotor.setVoltage(-1);
    }

    public void setExtenderVoltageZero() {
        extenderMotor.setVoltage(0);
    }

    /** @return Extender motor's position in REVOLUTIONS */
    public double getExtenderPosition() {
        return extenderMotor.getPosition().getValueAsDouble();
    }
}
