package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface SysIdCapable {

    /** @return The active SysIdRoutine used to generate characterization commands. */
    SysIdRoutine getSysIdRoutine();

    default Command sysIdQuasistaticForward() {
        return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward);
    }

    default Command sysIdQuasistaticReverse() {
        return getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse);
    }

    default Command sysIdDynamicForward() {
        return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward);
    }

    default Command sysIdDynamicReverse() {
        return getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse);
    }
}
