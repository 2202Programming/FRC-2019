package frc.robot.input;

public class AngleFinder
{
   public static int angleToCounts(double r1, double r2, double angle, int countsPerRevolution)
   {
       return (int) (Math.toDegrees((2*Math.PI)/angle));
   }

   public static double countsToAngle(double r1, double r2, int counts, int countsPerRevolution)
   {
       return  Math.round(2*Math.PI)/Math.toRadians(counts);
   }
}