package frc.robot.subsystems;

import static frc.robot.utilities.TargetingUtils.*;
import static frc.robot.utilities.FieldUtils.*;
import static frc.robot.utilities.HardwareUtils.*;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.DashboardManager;

public class Flywheel extends SubsystemBase implements SysIdCapable {
    private final Drivetrain drivetrain;
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    // Old m/s based: ks: 0.025659, kv: 0.33677, ka: 0.040121
    final Slot0Configs feedforwardConfig = new Slot0Configs().withKS(0.033378).withKV(0.10787).withKA(0.014195);

    private double currentFlywheelSpeed = 0;
    private double targetFlywheelSpeed = 0;

    // 5 seconds * 50 loops per second = 250 samples
    private final LinearFilter speedFilter = LinearFilter.movingAverage(250);
    private double averageFlywheelSpeed = 0;
    private double calculatedVoltage = 0;

    private final SysIdRoutine routine =
        SysIdCapable.createAngularRoutine(this, rightFlywheelMotor, this::setFlywheelMotorVoltages);

    public Flywheel(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        setMotorClockwisePositive(leftFlywheelMotor);
        applyGearRatio(1, leftFlywheelMotor, rightFlywheelMotor);
        applyFeedforward(feedforwardConfig, leftFlywheelMotor, rightFlywheelMotor);
        // @formatter:off
        DashboardManager.setupFlywheel(
            () -> currentFlywheelSpeed,
            () -> targetFlywheelSpeed,
            () -> averageFlywheelSpeed,
            () -> calculatedVoltage); // @formatter:on
    }

    @Override
    public void periodic() {
        averageFlywheelSpeed = speedFilter.calculate(currentFlywheelSpeed);
    }

    /** Sets both flywheel motors to the specified voltage (left voltage negated in constructor). */
    public void setFlywheelMotorVoltages(double volts) {
        leftFlywheelMotor.setVoltage(volts);
        rightFlywheelMotor.setVoltage(volts);
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSysIdRoutine'");
    }

    public double getCurrentFlywheelSpeed() {
        return currentFlywheelSpeed;
    }

    public double getTargetFlywheelSpeed() {
        return targetFlywheelSpeed;
    }

}
