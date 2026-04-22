package frc.robot.utilities;

/**
 * import static frc.robot.utilities.MathUtils.*;
 * 
 * Utility interface containing static math helper functions.
 */
public interface MathUtils {
    /**
     * @param value The double to round.
     * @param places The number of decimal places to round the double to. Must be an integer >= 1.
     * 
     * @return A double rounded to the specified number of decimal places.
     */
    static double round(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }
}
