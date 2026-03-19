package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DashboardManager;
import static frc.robot.Utilities.*;

public class Intake extends SubsystemBase implements SysIdCapable, MotionMagicCapable {
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

    // --- Motion Magic Configuration ---
    private final MotionMagicVoltage extenderMotionRequest = new MotionMagicVoltage(0);
    private final double ARM_STOWED_POSITION = 0.0;
    private final double ARM_DROPPED_POSITION = -0.3; // NEEDS TO BE CHECKED

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

    // --- SysId Configuration (Rollers Only) ---
    // ONLY UNCOMMENT THE ROUTINE CREATION LINE FOR THE MOTOR YOU WANT TO CHARACTERIZE.
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

        // --- Extender Motion Magic Setup ---
        // Initializing with zeros; actual values will be pushed via configureMotor()
        configureMotionMagic(extenderMotor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0);

        DashboardManager.setupIntake(() -> extenderMotorPosition);
    }

    @Override
    public void periodic() {
        extenderMotorPosition = getExtenderPosition();
    }

    public Command configureExtenderMotor() {
        return runOnce(() -> {
            configureMotionMagic(extenderMotor, DashboardManager.getExtenderkP(), DashboardManager.getExtenderkI(),
                DashboardManager.getExtenderkD(), DashboardManager.getExtenderkS(), DashboardManager.getExtenderkV(),
                DashboardManager.getExtenderkG(), DashboardManager.getExtenderMotionMagicCruiseVelocity(),
                DashboardManager.getExtenderMotionMagicAcceleration(), DashboardManager.getExtenderMotionMagicJerk());
        });
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
        return moveToPosition(ARM_DROPPED_POSITION, 0.02);
    }

    public Command raiseArmFinalImplementation() {
        return moveToPosition(ARM_STOWED_POSITION, 0.02);
    }

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
