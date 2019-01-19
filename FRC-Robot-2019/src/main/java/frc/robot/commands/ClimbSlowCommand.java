package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Command for climbing slow.
 */
public class ClimbSlowCommand extends Command {
  public ClimbSlowCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.climber.climbSlow();
  }

  @Override
  protected boolean isFinished() {
    // placeholder
    if (isTimedOut())
      return true;
    return false;
  }

  @Override
  protected void end() {
    Robot.climber.stopClimb();
  }
}