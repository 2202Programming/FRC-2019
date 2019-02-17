/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.input.Converter;

/**
 * A Lift subsystem.
 */
public class ArmSubsystem extends Subsystem {
  private WPI_TalonSRX armRotationMotor = new WPI_TalonSRX(RobotMap.ARM_ROTATION_TALON_CAN_ID);
  private WPI_TalonSRX armExtensionMotor = new WPI_TalonSRX(RobotMap.ARM_EXTENSTION_TALON_CAN_ID);
  private WPI_TalonSRX rotationEncoder;
  private WPI_TalonSRX extensionEncoder;
  private final double PHI_MAX = 157.0; //In Degrees, Positive is foward
  private final double PHI_MIN = 29.0; //In Degrees
  private final double COUNT_MAX = 54200.0; //In encoder counts (Proto Bot)

  public ArmSubsystem() {
    super("Arm");
    addChild("Arm Rotation Motor", armRotationMotor);
    addChild("Arm Extension Motor", armExtensionMotor);

    armRotationMotor.setNeutralMode(NeutralMode.Brake);
    armRotationMotor.config_kP(0, 0.17, 30);
    //armRotationMotor.config_kF(0, 0.002, 30);

    armExtensionMotor.setNeutralMode(NeutralMode.Brake);
    armExtensionMotor.config_kP(0, 0.1, 30);

    rotationEncoder = (WPI_TalonSRX) armRotationMotor;
    rotationEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rotationEncoder.setInverted(true);

    extensionEncoder = (WPI_TalonSRX) armExtensionMotor;
    extensionEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    extensionEncoder.setSelectedSensorPosition(0);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /**
   * Rotates the arm to a specific angle
   * @param angle the angle to rotate the arm to
   */
  public void setAngle(double angle) {
    double encoderPosition = convertAngleToCounts(angle);
    armRotationMotor.set(ControlMode.Position, encoderPosition);
  }

  /**
   * Convers the angle measure to encoder counts.
   * Used in the <code>setAngle</code> method.
   * @param angle the angle measure, in degrees, to convert.
   * @return the encoder counts used to set the angle of the arm.
   */
  private double convertAngleToCounts(double angle) {
    double counts = (PHI_MAX - angle) * COUNT_MAX / (PHI_MAX - PHI_MIN);
    return counts;
  }

  /**
   * Gets the angle at which the arm subsystem is rotated.
   * @return the angle of the arm, in radians.
   */
  public double getAngle() {
    return PHI_MAX - (rotationEncoder.getSelectedSensorPosition() / COUNT_MAX * (PHI_MAX - PHI_MIN));
  }

  public void logArmRotation() {
    SmartDashboard.putData((Sendable) armRotationMotor);
  }

  public void logArmExtnension() {
    SmartDashboard.putData((Sendable) armExtensionMotor);
  }

  public void rotateForward() {
    armRotationMotor.set(ControlMode.PercentOutput, 0.3);
  }

  public void rotateBackward() {
    armRotationMotor.set(ControlMode.PercentOutput, -0.3);
  }
  
  public void stopRotation() {
    armRotationMotor.set(0);
  }

  public TalonSRX getRotationEncoder() {
    return rotationEncoder;
  }
  
  /*
  public boolean rotationAtMin() {
    return rotationMinimumSwitch.get();
  }
  */

  public void extendToPosition(double distance) {
    int position = Converter.distanceToCounts(1.88, distance, 1024);
    armExtensionMotor.set(ControlMode.Position, position);
  } 

  public int getExtensionPosition() {
    return extensionEncoder.getSelectedSensorPosition();
  }

  public double getDistanceExtended() {
    return Converter.countsToDistance(1.88, getExtensionPosition(), 1024);
  }

  public void extend() {
    armExtensionMotor.set(0.3);
  }

  public void retract() {
    armExtensionMotor.set(-0.3);
  }

  public void stopExtension() {
    armExtensionMotor.set(0);
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
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void logTalons() {
    logTalon(armRotationMotor);
    logTalon(armExtensionMotor);
    //System.out.println("Encoder Count: " + rotationEncoder.getSelectedSensorPosition());
  }

  public void logTalon(WPI_TalonSRX talon) {
    SmartDashboard.putNumber(talon.getName() + " Current", talon.getOutputCurrent());
  }
}
