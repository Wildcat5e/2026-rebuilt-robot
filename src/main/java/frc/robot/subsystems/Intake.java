package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DashboardManager;
import static frc.robot.Utilities.*;

public class Intake extends SubsystemBase {
    /** Motor that is closer to the floor and scoops fuel into pusher. */
    private final TalonFX scooperMotor = new TalonFX(18);
    private final double SCOOPER_RADIUS = 0.06858 / 2;
    private final double SCOOPER_CIRCUMFERENCE = 2 * Math.PI * SCOOPER_RADIUS;

    /** Motor that shoots fuel into the robot */
    private final TalonFX pusherMotor = new TalonFX(16);
    private final double PUSHER_RADIUS = 0.058 / 2;
    private final double PUSHER_CIRCUMFERENCE = 2 * Math.PI * PUSHER_RADIUS;

    /** Motor that extends intake system outside of bumper. */
    private final TalonFX extenderMotor = new TalonFX(17);
    private double extenderMotorPosition = 0;

    // --- SysId Configuration ---
    // ONLY UNCOMMENT THE ROUTINE CREATION LINE FOR THE MOTOR YOU WANT TO CHARACTERIZE.
    // OTHERWISE, YOU RISK DAMAGING YOUR ROBOT.
    // MAKE SURE TO COMMENT OUT THE OTHER TWO BEFORE RUNNING ANY CHARACTERIZATION COMMANDS.

    // --SCOOPER MOTOR--
    private final SysIdRoutine routine =
        createLinearRoutine(this, scooperMotor, scooperMotor::setVoltage, SCOOPER_CIRCUMFERENCE);

    // --PUSHER MOTOR--
    // private final SysIdRoutine routine =
    //     createLinearRoutine(this, pusherMotor, pusherMotor::setVoltage, PUSHER_CIRCUMFERENCE);

    /** @return The SysIdRoutine used to generate characterization commands. */
    public SysIdRoutine getSysIdRoutine() {
        return routine;
    }

    public Intake() {
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
            scooperMotor.setVoltage(-scooperMotorVoltage);
            pusherMotor.setVoltage(pusherMotorVoltage);
        }, () -> {
            scooperMotor.setVoltage(0);
            pusherMotor.setVoltage(0);
        });
    }

    public Command dropArmFinalImplementation() {
        return new FunctionalCommand(
            // --initialize--
            () -> extenderMotor.setVoltage(-3),

            // --execute--
            () -> {},

            // --end--
            interrupted -> extenderMotor.setVoltage(0),

            // --isFinished--
            () -> {
                return getExtenderPosition() <= -.3; // Check sign
            },
            // --addRequirements--
            this); // Pass in Intake
    }

    public Command raiseArmFinalImplementation() {
        return new FunctionalCommand(
            // --initialize--
            () -> extenderMotor.setVoltage(3),

            // --execute--
            () -> {},

            // --end--
            interrupted -> extenderMotor.setVoltage(0),

            // --isFinished--
            () -> {
                return getExtenderPosition() >= 0; // Check sign
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

    public void spinPusher() {
        double pusherMotorVoltage = DashboardManager.getPusherMotorTestVoltage();
        pusherMotor.setVoltage(pusherMotorVoltage);
    }

    public void stopPusher() {
        pusherMotor.setVoltage(0);
    }

    public Command bumpExtenderUp() {
        return startEnd(() -> extenderMotor.setVoltage(6), () -> extenderMotor.setVoltage(0))
            .withName("Bump Extender Up");
    }

    public Command bumpExtenderDown() {
        return startEnd(() -> extenderMotor.setVoltage(1), () -> extenderMotor.setVoltage(0))
            .withName("Bump Extender Down");
    }

    /** @return Extender motor's position in REVOLUTIONS */
    public double getExtenderPosition() {
        return extenderMotor.getPosition().getValueAsDouble();
    }
}
