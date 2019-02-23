/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.arm.ArmZero;
import frc.robot.commands.util.MathUtil;


/**
 * Arm based lifter subsystem.
 * 
 *     PHI0 and L0  are the starting positons where encoders are zeroed.  Happens at teleOp init.
 *     So we expect to start system off at known location.
 * 
 *    2/20/19    DPL    updated constants from measured protobot
 *                      removed encoder zeros from init, leave that to zeroArm
 *                      moved logging from robot to here, removed public accesss to encoders
 *                      double checked math with non-zero initial start positons
 * 
 * 
 * 
 *    Arm gearing
 *              M:7:E:7:3:shaft:(12->30)  where M=motor, 7= 7:1, 3= 3:1  ( 12->30 ) pulley teeth (2.5)
 *              E: encoder 1024 counts/rev quad encoder
 * 
 *              ---->> 149.3333 counts/(deg arm)  WTH? measured ~824
 * 
 *    Ext gearing 
 *              M:5:7:E:shaft:(42:22:36):5mm pitch   
 *          36 teeth@5mm
 * 
 * 
 * 
 */
public class ArmSubsystem extends ExtendedSubSystem {
  private WPI_TalonSRX armRotationMotor = new WPI_TalonSRX(RobotMap.ARM_ROTATION_TALON_CAN_ID);
  private WPI_TalonSRX armExtensionMotor = new WPI_TalonSRX(RobotMap.ARM_EXTENSTION_TALON_CAN_ID);

  // Constants used by commands as measured
  public final double PHI0 = 157.0;    // degrees, starting position - encoder zero
  public final double PHI_MAX = 157.0; //In Degrees, Positive is foward, bottom front
  public final double PHI_MIN = 29.0;  //In Degrees, Near top front 
  //private final double COUNT_MAX = 54200.0; //In encoder counts (Proto Bot)
  private final double kCounts_per_deg = 824;  // 149.333;  // TODO:confirm //COUNT_MAX / (PHI_MAX - PHI_MIN);
  private final double kDeg_per_count = 1.0 / kCounts_per_deg;
  
  //Geometry of the arm's pivot point
  public final double PIVOT_TO_FRONT = 16.5; // inches pivot center to the frame  
  public final double MIN_PROJECTION = PIVOT_TO_FRONT + -6.5; //inches from pivot to close arm position
  public final double MAX_PROJECTION = PIVOT_TO_FRONT + Robot.kProjectConstraint; //

  // Extender phyiscal numbers - d
  public final double L0 = 7.69;                 // inches - starting point, encoder zero 
  public final double EXTEND_MIN = 0.0;          // inches
  public final double EXTEND_MAX = 37.0;         // inches - measured practice bot
  public final double ARM_BASE_LENGTH = 18.0;    //inches - measured practice bot (from pivot center) xg 2/16/19
  public final double ARM_PIVOT_HEIGHT = 30.25;  //inches - measured practice bot
  public final double WRIST_LENGTH = 7.75;       //inches -measured practice bot
  
  private final double EXTEND_COUNT_MAX = 22500; // measured practice bot
  private final double kCounts_per_in = EXTEND_COUNT_MAX / EXTEND_MAX;
  private final double kIn_per_count = 1.0 / kCounts_per_in;

  // Coupling between Phi and extension, as arm moves, so does d
  private final double k_dl_dphi = 0.032639; // inches per degree (measured practice bot 2/17/19)

  //talon controls
  final int PIDIdx = 0; //using pid 0 on talon
  final int TO = 30;    //timeout 30ms

  private short inversionConstant;

  /**
   * Creates a new arm/lift subsystem.
   */
  public ArmSubsystem() {
    super("Arm");
    addChild("Arm:Rot:Mtr", armRotationMotor);
    addChild("Arm:Ext:Mtr", armExtensionMotor);

    // Set Talon postion mode gains
    armRotationMotor.config_kP(0, 0.0 /* 0.8*/, 30);
    armExtensionMotor.config_kP(0, 0.0 /*0.6*/, 30);
    System.out.println("Warning - Arm motors have zeros for all Talon pid loops");

    // Arm
    armRotationMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armRotationMotor.setInverted(true);
    
    // Extension on power will be out. 
    armExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armExtensionMotor.setIntegralAccumulator(0, 0, 30);
    armExtensionMotor.setSensorPhase(true);
    armExtensionMotor.setInverted(true);

    zeroArm();  // will also get called on transition to teleOp, should arms be moved 

    inversionConstant = 1;
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /**
   * Zeros the arm's encoders - arm and extension are at starting point.
   * Used by commands at power up, or teleOp init.
   */
  public void zeroArm()
  {
    armExtensionMotor.setSelectedSensorPosition(0);
    armExtensionMotor.setIntegralAccumulator(0, PIDIdx, TO);
   // armExtensionMotor.configClosedloopRamp(0.25, TO);        //.25 seconds
   // armExtensionMotor.configContinuousCurrentLimit(10, TO);  //amps

    armRotationMotor.setSelectedSensorPosition(0);
    armRotationMotor.setIntegralAccumulator(0.0, PIDIdx, TO);
    //armRotationMotor.configClosedloopRamp(0.50, TO);        //.50 seconds
    //armRotationMotor.configContinuousCurrentLimit(10, TO);  //amps
  }

  /**
   * Rotates the arm to a specific angle
   * 
   * @param angle the angle to rotate the arm to
   */
  public void setAngle(double angle) {
    //If inverted, translate based on max angle backwards
    double counts = (PHI0 - inversionConstant*angle) * kCounts_per_deg;
    armRotationMotor.set(ControlMode.Position, counts);
  }

  /**
   * Gets the angle at which the arm subsystem is rotated.
   * @return the angle of the arm, in degrees.
   */
  public double getRealAngle() {
    //  return PHI_MAX - (rotationEncoder.getSelectedSensorPosition()  / COUNT_MAX * (PHI_MAX - PHI_MIN));
    double counts = armRotationMotor.getSelectedSensorPosition();
    double angle = PHI0 - counts * kDeg_per_count;
    return angle;
  }

  public double getAbsoluteAngle() {
    double counts = armRotationMotor.getSelectedSensorPosition();
    double angle = PHI0 - counts * kDeg_per_count;
    return inversionConstant * angle;
  }

  /**
   * Extends the extension to the length given. Compensate for the arm angle, phi,
   * so the desired extension is maintained.
   * 
   *  Because the encoders are zeroed at PHI0 and D0 they must be accounted for...
   *  At phi == phi0 there is no changes in length.
   * 
   * @param extendInch  (inches) to set the arm.
   */
  public void setExtension(double l) {
    double angle = getRealAngle();                     //current angle
    double compLen = ((angle - PHI0)*k_dl_dphi);   // ext due to rotation to compensate for
    double len = (l - L0) - compLen;               // net len to command

    //Make sure we limit to the range of the extension is capable
    if (len < EXTEND_MIN) {
      System.out.println("Arm:Extension below minimum.");
    }

    len = MathUtil.limit(len, EXTEND_MIN, EXTEND_MAX);
    double c = len * kCounts_per_in;
    armExtensionMotor.set(ControlMode.Position, c);
  }

  /**
   * Gets the extension length of the extension (l) in inches.
   * This is not the total arm lenght, it is just the extension.
   * @return extension (l), in inches.
   */
  public double getExtension() {
    int counts = armExtensionMotor.getSelectedSensorPosition();
    //   L0  + phi correction
    return (L0 + (getRealAngle() - PHI0) * k_dl_dphi) + (counts * kIn_per_count);
  }

  
  /**
   * Gets whether or not the arm is at the state of being the least extended it can.
   * @return <code>true</code> if the arm is at the minimum extension state, <code>false</code> otherwise.
   */
  public boolean extensionAtMin() {
    return armExtensionMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  /**
   * Gets whether or not the arm is at the state of being the most extended it can.
   * @return <code>true</code> if the arm is at the maximum extension state, <code>false</code> otherwise.
   */
  public boolean extensionAtMax() {
    return armExtensionMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new TeleopArmControlCommand());
    //Xander and derek don't want any default commands
  }

  /**
   * Gets the command used to zero the arm subsystem.
   * @return a new <code>ArmZero</code> command.
   */
  @Override
  public Command zeroSubsystem() {
    return new ArmZero();
  }

  public void invert() {
    inversionConstant *= -1;
  }

  public short getInversion() {
    return inversionConstant;
  }

  @Override
  public void log() {
    //SmartDashboard.putData((Sendable) armRotationMotor);
    //SmartDashboard.putData((Sendable) armExtensionMotor);
    SmartDashboard.putNumber("Arm:Phi(raw)", armRotationMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm:Phi(deg)", getAbsoluteAngle());
    SmartDashboard.putNumber("Arm:Ext(raw)", armExtensionMotor. getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm:Ext(in)",  getExtension());

    // don't have limit switches right now
    //SmartDashboard.putBoolean("Arm:Ext@Min", extensionAtMin());
    //SmartDashboard.putBoolean("Arm:Ext@Max", extensionAtMax());
  }

  /**
   * Logs the talons used in the arm subsystem, which are the rotation motor and the extension motor.
   */
  public void logTalons() {
    logTalon(armRotationMotor);
    logTalon(armExtensionMotor);
  }

  /**
   * Logs the given talon.
   * @param talon the WPI_TalonSRX to log.
   */
  private void logTalon(WPI_TalonSRX talon) {
    SmartDashboard.putNumber(talon.getName() + " Current", talon.getOutputCurrent());
    SmartDashboard.putNumber(talon.getName() + " Percent Output", talon.getMotorOutputPercent());
  }

}
