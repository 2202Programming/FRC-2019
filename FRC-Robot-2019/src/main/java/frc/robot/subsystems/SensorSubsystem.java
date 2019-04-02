package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

public class SensorSubsystem extends Subsystem {

  public static SerialPortSubsystem serialSubsystem;
  public static LimeLightSubsystem limeLight;
  private long logTimer;

  public SensorSubsystem() {
    serialSubsystem = new SerialPortSubsystem();
    limeLight = new LimeLightSubsystem();
    logTimer = System.currentTimeMillis();
  }

  public void processSensors() {
    limeLight.populateLimelight();
    serialSubsystem.processSerial();
  }

  public double getX() {
    return limeLight.getX();
  }

  public double getXAvg() {
    return limeLight.getXAvg();
  }

  public double getY() {
    return limeLight.getY();
  }

  public double getYAvg() {
    return limeLight.getYAvg();
  }

  public double getArea() {
    return limeLight.getArea();
  }

  public double getAreaAvg() {
    return limeLight.getAreaAvg();
  }

  public boolean hasTarget() {
    return limeLight.hasTarget();
  }

  public boolean hasTargetReliable() {
    return limeLight.hasTargetReliable();
  }

  public void disableLED() {
    limeLight.disableLED();
  }

  public void enableLED() {
    limeLight.enableLED();
  }

  public void log() {
      serialSubsystem.log();
      logTimer = System.currentTimeMillis();
      SmartDashboard.putNumber("Front Distance", getDistanceFront());
      SmartDashboard.putNumber("Back Distance", getDistanceBack());
      SmartDashboard.putNumber("Limelight Angle Distance", getLimelightDistanceByAngle());
      SmartDashboard.putNumber("Limelight Area Distance", getLimelightDistanceByArea());
    
  }

  public double getDistanceFront() // returns distance from target (either from Front Lidar or from lime light
                                   // depending on reliability), -1 if not available
  {
    double distance = 9999;
    double distance2 = 9999;
    double conversion = 0.001; // area to range in mm conversion
    double areaMax = 3; // Once target area is over this size, will check Lidar for reliability and use
                        // Lidar if available
    double errorConstant = 0.9;

    if (limeLight.hasTargetReliable() == true && limeLight.getAreaAvg() >= areaMax) {

      if (serialSubsystem.isReliable(RobotMap.LEFT_FRONT_LIDAR, errorConstant)
          && serialSubsystem.isReliable(RobotMap.RIGHT_FRONT_LIDAR, errorConstant)) {
        distance = Double.valueOf(serialSubsystem.getDistanceAvg(RobotMap.LEFT_FRONT_LIDAR));
        distance2 = Double.valueOf(serialSubsystem.getDistanceAvg(RobotMap.RIGHT_FRONT_LIDAR));

        return (distance + distance2) / 2;
      } else {

        return limeLight.getAreaAvg() * conversion;
      }
    } else {

      if (serialSubsystem.isReliable(RobotMap.LEFT_FRONT_LIDAR, errorConstant)
          && serialSubsystem.isReliable(RobotMap.RIGHT_FRONT_LIDAR, errorConstant)) {
        distance = Double.valueOf(serialSubsystem.getDistanceAvg(RobotMap.LEFT_FRONT_LIDAR));
        distance2 = Double.valueOf(serialSubsystem.getDistanceAvg(RobotMap.RIGHT_FRONT_LIDAR));

        return (distance + distance2) / 2;
      } else {
        return -1;
      }
    }
  }

  public double getDistanceBack() // returns distance from target using back Lidar, -1 if not reliable
  {
    double distance = 9999;
    double distance2 = 9999;
    double errorConstant = 0.9;

    if (serialSubsystem.isReliable(RobotMap.LEFT_BACK_LIDAR, errorConstant)
        && serialSubsystem.isReliable(RobotMap.RIGHT_BACK_LIDAR, errorConstant)) {
      distance = Double.valueOf(serialSubsystem.getDistanceAvg(RobotMap.LEFT_BACK_LIDAR));
      distance2 = Double.valueOf(serialSubsystem.getDistanceAvg(RobotMap.RIGHT_BACK_LIDAR));

      return (distance + distance2) / 2;
    } else {
      return -1;
    }

  }

  public double getLimelightDistanceByAngle() { // convert limelight Y angle to distance in mm
    double y = limeLight.getY();
    y = Math.toRadians(y);

    if (Math.tan(y) == 0)
      return 0;
    else
      return 10 * (48 / Math.tan(y));

  }

  public double getLimelightDistanceByArea() { // gets distance using area of target box
    double constant = 1 / 30325;
    double a = limeLight.getArea();
    if (constant * a == 0)
      return 0;
    else
      return (Math.sqrt(1 / (constant * a))) - 6;// 6 is magic number...

  }

  @Override
  public void initDefaultCommand() {
    // set default command
  }

  /**
   * @return the logTimer
   */
  public long getLogTimer() {
    return logTimer;
  }

}