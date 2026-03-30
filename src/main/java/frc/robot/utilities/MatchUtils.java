package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

/**
 * import static frc.robot.utilities.MatchUtils.*;
 */
public interface MatchUtils {

    enum MatchPhase {
        TRANSITION(130.0), SHIFT_1(105.0), SHIFT_2(80.0), SHIFT_3(55.0), SHIFT_4(30.0), ENDGAME(0.0);

        /** Measured in seconds remaining in the match. */
        public final double endTime;

        MatchPhase(double endTime) {
            this.endTime = endTime;
        }

        public static MatchPhase getPhase(double matchTime) {
            if (matchTime > 130) return TRANSITION;
            if (matchTime > 105) return SHIFT_1;
            if (matchTime > 80) return SHIFT_2;
            if (matchTime > 55) return SHIFT_3;
            if (matchTime > 30) return SHIFT_4;
            return ENDGAME;
        }
    }

    /** @return Whether or not the current Alliance's Hub Shift is Active. */
    static boolean isHubActive() {
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, and assume the Hub is active, as it's likely early in Teleop.
        if (gameData == null || gameData.isEmpty()) {
            DriverStation.reportWarning("Game Data is empty.", false);
            return true;
        }

        char firstChar = Character.toUpperCase(gameData.charAt(0));
        if (firstChar != 'R' && firstChar != 'B') {
            DriverStation.reportWarning("Game Data is invalid: " + gameData, false);
            return true;
        }

        // Shift 1 is active for Blue if Red won auto ('R'), or for Red if Blue won auto ('B')
        boolean redInactiveFirst = (firstChar == 'R');
        boolean shift1Active = Robot.isBlueAlliance ? redInactiveFirst : !redInactiveFirst;

        MatchPhase currentPhase = MatchPhase.getPhase(DriverStation.getMatchTime());

        return switch (currentPhase) {
            case TRANSITION -> true;
            case SHIFT_1 -> shift1Active;
            case SHIFT_2 -> !shift1Active;
            case SHIFT_3 -> shift1Active;
            case SHIFT_4 -> !shift1Active;
            case ENDGAME -> true;
        };
    }

    /** @return The number of seconds remaining in the current Alliance's Hub Shift. */
    static double getHubShiftTimeRemaining() {
        if (!DriverStation.isTeleopEnabled()) return 0.0;

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) return 0.0;

        MatchPhase currentPhase = MatchPhase.getPhase(matchTime);
        return matchTime - currentPhase.endTime;
    }
}
