package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardManager;

import static frc.robot.Utilities.applyGearRatio;

public class Intake extends SubsystemBase {
    /**
     * Motor that shoots fuel into the robot
     */
    private final TalonFX pusherMotor           = new TalonFX(16);
    /**
     * Motor that extends intake system outside of bumper.
     */
    private final TalonFX extenderMotor         = new TalonFX(17);
    /**
     * Motor that is closer to the floor and scoops fuel into other set of wheels.
     */
    private final TalonFX scooperMotor          = new TalonFX(18);
    private       double  extenderMotorPosition = 0;

    public Intake() {
        applyGearRatio(scooperMotor, 1);
        applyGearRatio(pusherMotor, 1);
        applyGearRatio(extenderMotor, 36);
        extenderMotor.setNeutralMode(NeutralModeValue.Brake);
        DashboardManager.setupIntake(() -> extenderMotorPosition, () -> scooperMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        extenderMotorPosition = getExtenderPosition();
    }

    public Command spinIntakeMotors() {
        return startEnd(() -> {
            double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
            double pusherMotorVoltage  = DashboardManager.getPusherMotorTestVoltage();
            scooperMotor.setVoltage(-scooperMotorVoltage);
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> {
            scooperMotor.setVoltage(3); // TODO: Why is this 3 and not 0?
            pusherMotor.setVoltage(0);
        });
    }


    /**
     * TURN ON INTAKE TO TAKE IN FUEL COMMAND <br>
     * Currently plan to bind to a while true button, but may be easier to have it run on toggle or for the entire match
     */
    public Command testScooper() {
        return startEnd(() -> {
            double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
            scooperMotor.setVoltage(-scooperMotorVoltage);
        }, () -> scooperMotor.set(0));
    }

    public Command reverseScooper() {
        return startEnd(() -> scooperMotor.setVoltage(3), () -> scooperMotor.setVoltage(0)).withName("Reverse Scooper");
    }

    public Command testPusher() {
        return startEnd(() -> {
            double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> pusherMotor.set(0));
    }

    public Command reversePusher() {
        return startEnd(() -> pusherMotor.setVoltage(3), () -> pusherMotor.setVoltage(0)).withName("Reverse Pusher");
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
     * @return Extender motor's position in REVOLUTIONS
     */
    public double getExtenderPosition() {
        return extenderMotor.getPosition().getValueAsDouble();
    }
}
