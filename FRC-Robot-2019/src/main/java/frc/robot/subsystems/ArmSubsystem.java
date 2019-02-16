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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.arm.TeleopArmControlCommand;
import frc.robot.input.Converter;

/**
 * A Lift subsystem.
 */
public class ArmSubsystem extends Subsystem {
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

  public ArmSubsystem() {
    super("Arm");
    addChild("Arm Rotation Motor", armRotationMotor);
    addChild("Arm Extension Motor", armExtensionMotor);

    armRotationMotor.config_kP(0, 0.00015, 30);
    //armRotationMotor.config_kF(0, 0.002, 30);

    armExtensionMotor.config_kP(0, 0.1, 30);

    rotationEncoder = (WPI_TalonSRX) armRotationMotor;
    rotationEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    extensionEncoder = (WPI_TalonSRX) armExtensionMotor;
    extensionEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    extensionEncoder.setSelectedSensorPosition(0);
    curAngle = PHI_MAX;
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

  private double convertAngleToCounts(double angle) {
    double counts = (PHI_MAX - angle) * COUNT_MAX / (PHI_MAX - PHI_MIN);
    return counts;
  }

  public void logArmRotation() {
    SmartDashboard.putData((Sendable) armRotationMotor);
  }

  public void logArmExtnension() {
    SmartDashboard.putData((Sendable) armExtensionMotor);
  }

  public double getAngle() {
    return PHI_MAX + Converter.countsToAngle(1.88, 2.05, rotationEncoder.getSelectedSensorPosition(), 1024*7);
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
    setDefaultCommand(new TeleopArmControlCommand());
  }

  public void resetRotationEncoder() {
    rotationEncoder.setSelectedSensorPosition(0);
  }

  public void resetExtensionEncoder() {
    extensionEncoder.setSelectedSensorPosition(0);
  }

  public void logTalons() {
    logTalon(armRotationMotor);
    logTalon(armExtensionMotor);
    //System.out.println("Encoder Count: " + rotationEncoder.getSelectedSensorPosition());
  }

  public void logTalon(WPI_TalonSRX talon) {
    SmartDashboard.putNumber(talon.getName() + " Current", talon.getOutputCurrent());
    SmartDashboard.putNumber(talon.getName() + " Percent Output", talon.getMotorOutputPercent());
  }
}
