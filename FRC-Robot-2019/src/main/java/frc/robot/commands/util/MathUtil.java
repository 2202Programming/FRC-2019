package frc.robot.commands.util;

/**
 *  Common helper functions
 */
public class MathUtil {

    public static double limit(double x, double min, double max) {
        return Math.max(min, Math.min(x, max) );
    }

}
