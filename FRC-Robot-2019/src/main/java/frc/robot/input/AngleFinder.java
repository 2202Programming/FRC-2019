package frc.robot.input;

public class AngleFinder
{

   public double calculateAngle(double CircumferenceRatio, int encoderCount)
   {
       return Math.toDegrees((2*Math.PI)/encoderCount);
   }


   //Not exact calculations
   public int calculateCount(double r1, double r2, double angle)
   {
       return (int) ((2*Math.PI)/Math.toRadians(angle));
   }
}