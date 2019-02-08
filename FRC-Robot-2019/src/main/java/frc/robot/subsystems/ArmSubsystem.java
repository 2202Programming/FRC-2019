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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.input.AngleFinder;

/**
 * A Lift subsystem.
 */
public class ArmSubsystem extends Subsystem {
  private WPI_TalonSRX armRotationMotor = new WPI_TalonSRX(RobotMap.ARM_ROTATION_TALON_CAN_ID);
  private WPI_TalonSRX armExtensionMotor = new WPI_TalonSRX(RobotMap.ARM_EXTENSTION_TALON_CAN_ID);
  private WPI_TalonSRX rotationEncoder;
  private WPI_TalonSRX extensionEncoder;
  private DigitalInput rotationMinimumSwitch = new DigitalInput(RobotMap.ARM_MIN_ROTATION_SENSOR_PIN);
  private DigitalInput extensionMinimumSwitch = new DigitalInput(RobotMap.ARM_MIN_EXTENSION_SENSOR_PIN);

  public ArmSubsystem() {
    super("Arm");
    addChild("Arm Rotation Motor", armRotationMotor);
    addChild("Arm Extension Motor", armExtensionMotor);
    addChild("Arm Rotation Limit Switch", rotationMinimumSwitch);
    addChild("Arm Extension Limit Switch", extensionMinimumSwitch);

    rotationEncoder = (WPI_TalonSRX) armRotationMotor;
    rotationEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    extensionEncoder = (WPI_TalonSRX) armExtensionMotor;
    extensionEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /**
   * Rotates the arm to a specific angle
   * @param angle the angle to rotate the arm to
   */
  public void rotateToPosition(double angle) {
    int encoderPosition = AngleFinder.angleToCounts(1.88, 2.005, angle, 1024);
    armRotationMotor.set(ControlMode.Position, encoderPosition);
  }

  public TalonSRX getArmRotationEncoder() {
    return rotationEncoder;
  }

  public TalonSRX getArmExtensionEncoder() {
    return extensionEncoder;
  }

  public void rotateForward() {
    armRotationMotor.set(0.3);
  }

  public void rotateBackward() {
    armRotationMotor.set(-0.3);
  }
  
  public void stopRotation() {
    armRotationMotor.set(0);
  }

  public TalonSRX geRotationEncoder() {
    return rotationEncoder;
  }

  public boolean rotationAtMin() {
    return rotationMinimumSwitch.get();
  }

  public void extendToPosition(double position) {
    armExtensionMotor.set(ControlMode.Position, position);
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
    return extensionMinimumSwitch.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void logTalons() {
    logTalon(armRotationMotor);
    logTalon(armExtensionMotor);
  }

  public void logTalon(WPI_TalonSRX talon) {
    SmartDashboard.putNumber(talon.getName() + " Current", talon.getOutputCurrent());
  }
}
