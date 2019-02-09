package frc.robot.input;

public class AngleFinder {
    /**
     * Converts angular displacement to encoder counts
     * 
     * @param r1                  The radius of the pulley the motor output is bound 
     *                            to
     * @param r2                  The radius of the pully the output shaft is bound
     *                            to
     * @param angle               The measured angular displacement in radians
     * @param countsPerRevolution The number of counts per full revolution
     * @return The linear displacement in encoder counts‰
     */
    public static int angleToCounts(double r1, double r2, double angle, int countsPerRevolution) {
        return angleToCounts(r1 / r2, angle, countsPerRevolution);
    }

    /**
     * Converts angular displacement to encoder counts 
     * 
     * Formula is Counts = θ * Counts Per Revolution / Ratio
     * 
     * @param ratio               The ratio of linear displacement over angular
     *                            displacement
     * @param angle               The measured angular displacement in radians
     * @param countsPerRevolution The number of counts per full revolution
     * @return The linear displacement in encoder counts
     */
    public static int angleToCounts(double ratio, double angle, int countsPerRevolution) {
        return (int) Math.round(Math.toRadians(angle) * countsPerRevolution / ratio);
    }

    /**
     * Converts encoder counts to an angular displacement
     * 
     * @param r1                  The radius of the pulley the motor output is bound
     *                            to
     * @param r2                  The radius of the pully the output shaft is bound
     *                            to
     * @param counts              The number of counts measured
     * @param countsPerRevolution The number of counts per full revolution
     * @return The angular displacement in radians
     */
    public static double countsToAngle(double r1, double r2, int counts, int countsPerRevolution) {
        return countsToAngle(r1 / r2, counts, countsPerRevolution);
    }

    /**
     * Converts encoder counts to an angle in degrees 
     * 
     * Formula is θ = Counts / Counts Per Revolution * Ratio
     * 
     * @param ratio               The ratio of linear displacement over angular
     *                            displacement
     * @param counts              The number of counts measured
     * @param countsPerRevolution The number of counts per full revolution
     * @return The angular displacement in degrees
     */
    public static double countsToAngle(double ratio, int counts, int countsPerRevolution) {
        return Math.toDegrees((double) counts / countsPerRevolution * ratio);
    }
}