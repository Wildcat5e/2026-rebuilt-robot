package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

/** Used for Robot Setup. Lots of static methods and variables */
class Setup {
    /** The only instance of Drivetrain. */
    static final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    /** The only instance of the Xbox Controller. */
    static final CommandXboxController joystick = new CommandXboxController(0);

    static void configureBindings() {
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

}
