package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Command for climbing fast.
 */
public class ClimbFastCommand extends Command {
  
  public ClimbFastCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.climber.climbFast();
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
    Robot.climber.stopClimb();
  }
}
