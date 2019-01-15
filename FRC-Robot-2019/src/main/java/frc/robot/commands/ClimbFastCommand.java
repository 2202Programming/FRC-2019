package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ClimbFastCommand extends Command {
  public ClimbFastCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
  }
  protected void initialize() {
    Robot.climber.climbFast();
  }
  protected boolean isFinished() {
    return true;
  }
}
