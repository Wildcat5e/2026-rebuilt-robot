package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardManager;
import static frc.robot.Utilities.*;

public class Intake extends SubsystemBase {
    /** Motor that is closer to the floor and scoops fuel into other set of wheels. */
    private final TalonFX scooperMotor = new TalonFX(0);
    /** Motor that shoots fuel into the robot */
    private final TalonFX pusherMotor = new TalonFX(0);
    /** Motor that extends intake system outside of bumper. */
    private final TalonFX extenderMotor = new TalonFX(0);
    private double extenderMotorPosition = 0;

    public Intake() {
        // GEAR RATIO IS 36, BUT MECHANICAL RATIO IS 48
        applyGearRatio(scooperMotor, 1);
        applyGearRatio(pusherMotor, 1);
        applyGearRatio(extenderMotor, 36);
        DashboardManager.setupIntake(() -> extenderMotorPosition);
    }

    @Override
    public void periodic() {
        extenderMotorPosition = getExtenderPosition();
    }

    public Command spinIntakeMotors() {
        return startEnd(() -> {
            double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
            double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
            scooperMotor.setVoltage(scooperMotorVoltage);
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> {
            scooperMotor.setVoltage(0);
            pusherMotor.setVoltage(0);
        });
    }

    public Command dropArmFinalImplementation() {
        return new FunctionalCommand(
            // --initialize--
            () -> extenderMotor.setVoltage(3),

            // --execute--
            () -> {},

            // --end--
            interrupted -> extenderMotor.setVoltage(0),

            // --isFinished--
            () -> {
                return getExtenderPosition() >= .3; // Check sign
            },
            // --addRequirements--
            this); // Pass in Intake
    }

    /**
     * TURN ON INTAKE TO TAKE IN FUEL COMMAND <br>
     * Currently plan to bind to a while true button, but may be easier to have it run on toggle or for the entire match
     */
    public Command testScooper() {
        return startEnd(() -> {
            double scooperMotorVoltage = DashboardManager.getScooperMotorTestVoltage();
            scooperMotor.setVoltage(scooperMotorVoltage);
        }, () -> scooperMotor.set(0));
    }

    public Command testPusher() {
        return startEnd(() -> {
            double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> pusherMotor.set(0));
    }

    public Command testExtender() {
        return startEnd(() -> {
            double extenderMotorVoltage = DashboardManager.getExtenderMotorTestVoltage();
            extenderMotor.setVoltage(extenderMotorVoltage);
        }, () -> extenderMotor.setVoltage(0));
    }

    /** @return Extender motor's position in REVOLUTIONS */
    public double getExtenderPosition() {
        return extenderMotor.getPosition().getValueAsDouble();
    }
}
