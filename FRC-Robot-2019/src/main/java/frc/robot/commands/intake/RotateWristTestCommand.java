package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * This command is intended to be used for testing purposes.
 * This command turns the wrist from zero, to positive, then to zero, then to negative, then to zero.
 * 
 * @author Kevin Li
 */
public class RotateWristTestCommand extends Command {
  private XboxController ctrl = Robot.m_oi.getAssistantController();
  private double[] positions = {-30, 0, 30, 0};
  private int currentIndex = 0;

  public RotateWristTestCommand() {
    requires(Robot.intake);
  }

  @Override
  protected void initialize() {
    Robot.intake.setAngle(0);
  }

  @Override
  protected void execute() {
    if (ctrl.getAButtonReleased()) { // TODO: change to respective button
      Robot.intake.setAngle(positions[currentIndex]);
      currentIndex++;
      if (currentIndex > positions.length) currentIndex = 0;
    }
    
  }

  @Override
  protected boolean isFinished() {
    return true; // TODO: change
  }
  
  @Override
  protected void end() {
    super.end();
  }

  @Override
  protected void interrupted() {
    super.interrupted();
  }
}