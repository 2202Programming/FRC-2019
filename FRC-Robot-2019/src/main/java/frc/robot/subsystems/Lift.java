/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * A Lift subsystem.
 */
public class Lift extends PIDSubsystem {
  private WPI_TalonSRX liftMotor;
  private int setPosition;

  public static final int PLACEHOLDER = 0;
  public static final double P = 0;
  public static final double I = 0;
  public static final double D = 0;

  public Lift() {
    super("Lift", P, I, D);
    liftMotor = new WPI_TalonSRX(PLACEHOLDER);
    addChild("Lift Motor", liftMotor);
    setPosition = 0;
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  @Override
  protected void usePIDOutput(double output) {
    liftMotor.pidWrite(output);
  }
  @Override
  protected double returnPIDInput() {
      return 0;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
