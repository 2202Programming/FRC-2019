/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem.Position;

public class GripperPositionCommand extends Command {
  double timeout;
  double height;
  double projx;
  double error;
  
  // Phyical values from sub-systems as needed
  Position armPosition;

  public GripperPositionCommand(double height, double projx, double error, double timeout) {
      this.height = height;
      this.projx = projx;
      this.timeout = timeout;
      this.error = Math.abs(error);
  }

  @Override
  protected void initialize() {
      setTimeout(timeout);
      Robot.m_cmdMgr.cmdPosition(height, projx); // sets the CommandManagers h/x output
      armPosition = Robot.arm.getArmPosition();
  }

  @Override
  protected void execute() {
    armPosition = Robot.arm.getArmPosition();
    //Robot.m_cmdMgr.cmdPosition(height, projx); // once should be fine
  }

  @Override
  protected boolean isFinished() {
      double h_err = Math.abs(armPosition.height - height);
      double x_err = Math.abs(armPosition.projection - projx);
      boolean posGood = (h_err < error) && (x_err < error);
      return posGood || isTimedOut();
  }
}