/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ClimbZero extends CommandBase {

  /**
   * Creates an ClimbZero command. This command zeros the climbers encoder.
   */
  public ClimbZero() {
    addRequirements(Robot.climber);
  }

  /**
   * Zeros encoders
   */
  @Override
 public void initialize() {
    Robot.climber.zeroClimber();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}