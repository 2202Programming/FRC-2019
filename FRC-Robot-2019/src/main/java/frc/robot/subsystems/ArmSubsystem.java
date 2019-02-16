/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.arm.TeleopArmControlCommand;
import frc.robot.input.Converter;
//import frc.robot.triggers.MotorOverPowerShutdown;

/**
 * A Lift subsystem.
 */
public class ArmSubsystem extends ExtendedSubSystem {
  private WPI_TalonSRX armRotationMotor = new WPI_TalonSRX(RobotMap.ARM_ROTATION_TALON_CAN_ID);
  private WPI_TalonSRX armExtensionMotor = new WPI_TalonSRX(RobotMap.ARM_EXTENSTION_TALON_CAN_ID);
  private WPI_TalonSRX rotationEncoder;
  private WPI_TalonSRX extensionEncoder;
  private final double PHI_MAX = 145.0; //In Degrees, Positive is foward
  private final double PHI_MIN = 32.0; //In Degrees
  private final double COUNT_MAX = -13600.0; //In encoder counts (Proto Bot)
  public final double ARM_HEIGHT = 29.75; //In Inches
  public final double MIN_ARM_LENGTH = 30; //TODO: Find real value in inches
  public final double MAX_ARM_LENGTH = 68.0; //TODO: Find real value in inches
  public final double MIN_PROJECTION = 15.0; //TODO: Find real value in inches
  public final double MAX_PROJECTION = 45.0; //TODO: Find real value in inches
  private double curAngle;

  public final double EXTEND_MIN = 0.0; // inches
  public final double EXTEND_MAX = 38.0; // inches - measured
  private final double EXTEND_COUNT_MAX = 26400; // measured
  private final double kCounts_per_in = EXTEND_COUNT_MAX / EXTEND_MAX;
  private final double kIn_per_count = 1.0 / kCounts_per_in;

  public final double PHI_MAX = 145.0; // degrees, Positive is foward
  public final double PHI_MIN = 32.0; // degrees
  private final double COUNT_MAX = -13600.0; // encoder counts (Proto Bot - measured)

  //talon controls
  final int PIDIdx = 0; //using pid 0 on talon
  final int TO = 30;    //timeout 30ms

  public ArmSubsystem() {
    super("Arm");
    addChild("Arm Rot M", armRotationMotor);
    addChild("Arm Ext M", armExtensionMotor);

    armRotationMotor.config_kP(0, 0.00015, 30);
    //armRotationMotor.config_kF(0, 0.002, 30);

    armExtensionMotor.config_kP(0, 0.1, 30);

    rotationEncoder = (WPI_TalonSRX) armRotationMotor;
    rotationEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rotationEncoder.setSelectedSensorPosition(0);

    // dpl this didn't work - rotationEncoder.setInverted(true);

    // Assumes extension at zero on power up.
    extensionEncoder = (WPI_TalonSRX) armExtensionMotor;
    extensionEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    armExtensionMotor.setSelectedSensorPosition(0);
    armExtensionMotor.setIntegralAccumulator(0, 0, 30);
    armExtensionMotor.setSensorPhase(false);
    // safety triggers
    // MotorOverPowerShutdown opsExt =
    // new MotorOverPowerShutdown(this.armExtensionMotor, 20.0, 0.5);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //used by commands at power up
  public void zeroArm()
  {

    armExtensionMotor.setSelectedSensorPosition(0);
    armExtensionMotor.setIntegralAccumulator(0, PIDIdx, TO);

    armRotationMotor.setSelectedSensorPosition(0);
    armRotationMotor.setIntegralAccumulator(0.0, PIDIdx, TO);
  }

  /**
   * Rotates the arm to a specific angle
   * 
   * @param angle the angle to rotate the arm to
   */
  public void setAngle(double angle) {
    double encoderPosition = convertAngleToCounts(angle);
    armRotationMotor.set(ControlMode.Position, encoderPosition);
  }

  private double convertAngleToCounts(double angle) {
    double counts = (PHI_MAX - angle) * COUNT_MAX / (PHI_MAX - PHI_MIN);
    return counts;
  }

  public double getAngle() {
    return PHI_MAX - Converter.countsToAngle(1.88, 2.05, rotationEncoder.getSelectedSensorPosition(), 1024 * 7);
  }

  public TalonSRX getRotationEncoder() {
    return rotationEncoder;
  }

  public void setExtension(double extendInch) {
    double c = extendInch * kCounts_per_in;
    armExtensionMotor.set(ControlMode.Position, c);
  }

  // inches
  public double getExtension() {
    int counts = extensionEncoder.getSelectedSensorPosition();
    return counts * kIn_per_count;
    // return Converter.countsToDistance(0.94, counts, 1024);
  }

  public TalonSRX getExtensionEncoder() {
    return extensionEncoder;
  }

  public boolean extensionAtMin() {
    return armExtensionMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean extensionAtMax() {
    return armExtensionMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopArmControlCommand());
  }

  public void resetRotationEncoder() {
    rotationEncoder.setSelectedSensorPosition(0);
  }

  public void resetExtensionEncoder() {
    extensionEncoder.setSelectedSensorPosition(0);
  }

  @Override
  public Command zeroSubsystem() {
    return new ArmZero();
  }

  @Override
  public void log() {
    SmartDashboard.putData((Sendable) armRotationMotor);
    SmartDashboard.putData((Sendable) armExtensionMotor);
  }

  public void logTalons() {
    logTalon(armRotationMotor);
    logTalon(armExtensionMotor);
    System.out.println("Encoder Count: " + rotationEncoder.getSelectedSensorPosition());
  }
private void logTalon(WPI_TalonSRX talon) {
    SmartDashboard.putNumber(talon.getName() + " Current", talon.getOutputCurrent());
    SmartDashboard.putNumber(talon.getName() + " Percent Output", talon.getMotorOutputPercent());
  }

}
