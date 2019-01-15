package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ClimbSlowCommand extends Command {
  public ClimbSlowCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
  }
  protected void initialize() {
      Robot.climber.climbSlow();
  }
  protected boolean isFinished() {
    return true;
  }
}