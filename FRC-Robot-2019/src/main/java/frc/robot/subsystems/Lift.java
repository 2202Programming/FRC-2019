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

  /**
   * The device id of the lift motor.
   */
  public static final int PLACEHOLDER = 0; // TODO: Change constant name to something more descriptive
  /**
   * The proportional value of the lift subsystem.
   */
  public static final double P = 0;
  /**
   * The integral value of the lift subsystem.
   */
  public static final double I = 0;
  /**
   * The derivative value of the lift subsystem.
   */
  public static final double D = 0;

  /**
   * Creates a lift object.
   * The proportional, integral, and derivative values are zero, as is the device id of the lift motor.
   */
  public Lift() {
    super("Lift", P, I, D);
    liftMotor = new WPI_TalonSRX(PLACEHOLDER);
    addChild("Lift Motor", liftMotor);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /**
   * Sends the output of the pid loop to the lift motor.
   * @param output the output of the pid loop.
   */
  @Override
  protected void usePIDOutput(double output) {
    liftMotor.pidWrite(output);
  }

  @Override
  protected double returnPIDInput() {
      return 0; // TODO: Should the pid loop input be 0?
  }

  /**
   * Sets the lift motor to move upwards at a speed of 0.5.
   */
  public void runUp() {
    liftMotor.set(0.5);
  }

  /**
   * Sets the lift motor to move downwards at a speed of 0.5.
   */
  public void runDown() {
    liftMotor.set(-0.5);
  }

  /**
   * Stops the lift motor.
   */
  public void stop() {
    liftMotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  
}
