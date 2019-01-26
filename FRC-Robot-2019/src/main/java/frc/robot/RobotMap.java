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
  public static final int FL_TALON_CAN_ID = 11;
  public static final int ML_TALON_CAN_ID = 12;
  public static final int BL_TALON_CAN_ID = 13;
  public static final int FR_TALON_CAN_ID = 14;
  public static final int MR_TALON_CAN_ID = 15;
  public static final int BR_TALON_CAN_ID = 16;
  public static final int GEARSHIFT_SOLENOID_CAN_ID = 2;
  public static final int CLIMBER_SOLENOID_CAN_ID = 17;

  public static final int CLIMBER_MOTOR_PIN = 4;
  public static final int LEFT_INTAKE_MOTOR_PIN = 5;
  public static final int RIGHT_INTAKE_MOTOR_PIN = 6;

  public static final int INTAKE_PHOTOGATE_CHANNEL = 4;
}
