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
  //values in inches
  public static final double ENCODER_RIGHT_DISTANCE_PER_PULSE = 0.002396219298;
  public static final double ENCODER_LEFT_DISTANCE_PER_PULSE = 0.002399087014;
  public static final int ENCODER_COUNTS_PER_REVOLUTION = 8192; //Double check this
  public static final double WHEEL_RADIUS = 3;

  public static final int FL_TALON_CAN_ID = 11;
  public static final int ML_TALON_CAN_ID = 12;
  public static final int BL_TALON_CAN_ID = 13;
  public static final int FR_TALON_CAN_ID = 14;
  public static final int MR_TALON_CAN_ID = 15;
  public static final int BR_TALON_CAN_ID = 16;

  public static final int ENCODER_LEFT_PIN_1 = 0;
  public static final int ENCODER_LEFT_PIN_2 = 1;
  public static final int ENCODER_RIGHT_PIN_1 = 2;
  public static final int ENCODER_RIGHT_PIN_2 = 3;

  public static final int GEARSHIFT_SOLENOID_CAN_ID = 2;
  public static final int GEARSHIFTUP_SOLENOID_ID = 1;
  public static final int GEARSHIFTDOWN_SOLENOID_ID = 3;
  
  //Arm
  public static final int ARM_ROTATION_TALON_CAN_ID = 20;
  public static final int ARM_MIN_ROTATION_SENSOR_PIN = 4;
  public static final int ARM_ROTATION_ENCODER_PIN_0 = 5;
  public static final int ARM_ROTATION_ENCODER_PIN_1 = 6;

  public static final int ARM_EXTENSTION_TALON_CAN_ID = 21;
  public static final int ARM_MIN_EXTENSION_SENSOR_PIN = 7;
  public static final int ARM_EXTENSION_ENCODER_PIN_0 = 8;
  public static final int ARM_EXTENSION_ENCODER_PIN_1 = 9;
  
  //Cargo Trap
  public static final int TRAP_ARMS_PCM_ID = 1;
  public static final int CARGO_SENSOR_DIO_PORT = -1;
  public static final int TRAP_RETRACT_PCM_ID = 2;

  //End Effector
  public static final int WRIST_SERVO_PWM_CH = 0;
  public static final int CARGO_SWITCH_MXP_CH = 11;
  public static final int VACUUM_SPARK_PIN = 1;
  public static final int RELEASE_SOLENOID_ID = 3;
  public static final int GENATOR_ID = 4;

  //Climber
  public static final int FOOT_EXTEND_SPARK_PIN = 2;
  public static final int ROLLER_SPARK_PIN = 3;
  public static final int RATCHET_SELECTOR_ID = 5;
}
