package frc.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 * 
 *  This should match the Goolge docs resource assignments found here:
 *     https://docs.google.com/spreadsheets/d/1wAw8BCPKs3-sEif2DAuiti0Movgp9sCG6e_4lc_bVIc/edit#gid=0
 * 
 * Changes:
 * 2/10/2019 Derek L    DoubleSolenoids used; PWM & PCM conventions for Spark/Servos/Solenoids
 *                      Intake Release/Hold moved to PCM 5/6 to better organize wires
 *                      GearShift PCM 0-1
 * 
 * 
 */
public class RobotMap {
  
  //CAN ID for non-motor devices
  public static final int CAN_PDP_ID  = 0;
  public static final int CAN_PCM1_ID = 1;
  public static final int CAN_PCM2_ID = 2;

  //Drive Train
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

  public static final int GEARSHIFT_PMC_ID = CAN_PCM1_ID;
  public static final int GEARSHIFTUP_SOLENOID_PCM = 0;
  public static final int GEARSHIFTDOWN_SOLENOID_PCM = 1;
  
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
  public static final int TRAP_PCM_ID = CAN_PCM1_ID;
  public static final int TRAP_DEPLOY_PCM = 2; 
  public static final int TRAP_RETRACT_PCM = 3;
  public static final int TRAP_ARMS_OPEN_PCM = 4; 
  public static final int TRAP_ARMS_CLOSE_PCM = 5;
  public static final int TRAP_CARGO_SENSOR_DIO = 30;

  //End Effector IntakeSubsystem
  public static final int INTAKE_WRIST_SERVO_PWM = 0;
  public static final int INTAKE_VACUUM_SPARK_PWM = 1;    
  public static final int INTAKE_CARGO_SWITCH_MXP_CH = 11;
  public static final int INTAKE_PCM_ID = CAN_PCM1_ID;
  public static final int INTAKE_RELEASE_SOLENOID_PCM = 6;   
  public static final int INTAKE_HOLD_SOLENOID_PCM = 7;      

  //Climber
  public static final int CLIMB_FOOT_SPARK_PWM = 2;
  public static final int CLIMB_ROLLER_SPARK_PWM = 3;
  public static final int CLIMB_RATCHET_PCM_ID = CAN_PCM2_ID;  //Second PCM
  public static final int CLIMB_RATCHET_DOWN_PCM = 0;
  public static final int CLIMB_RATCHET_UP_PCM = 1;

  //LIDAR sensors
  public static final int LEFT_FRONT_LIDAR = 1;
  public static final int RIGHT_FRONT_LIDAR = 2;
  
}
