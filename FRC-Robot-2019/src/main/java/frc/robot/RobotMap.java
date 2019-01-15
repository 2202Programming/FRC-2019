/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static final int FL_MOTOR_PIN = 3;
  public static final int BL_MOTOR_PIN = 2;
  public static final int FR_MOTOR_PIN = 1;
  public static final int BR_MOTOR_PIN = 0;
  public static final int CLIMBER_MOTOR_PIN = 4;
  public static final int LEFT_INTAKE_MOTOR_PIN = 5;
  public static final int RIGHT_INTAKE_MOTOR_PIN = 6;
}
