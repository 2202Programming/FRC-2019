package frc.robot.commands; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;

public class RotateWristUpCommand extends Command{

    public RotateWristUpCommand(){
        requires(Robot.intake);
    }

    @Override
    protected void initialize() {
        Robot.intake.stopWrist();
    }


  @Override
  protected void execute() {
      Robot.intake.runWristUp();

  }

  @Override
  protected boolean isFinished() {
    return Robot.m_oi.getController1().getRawButtonReleased(XboxControllerButtonCode.START.getCode());
  }

 
  @Override
  protected void end() {
      Robot.intake.stopWrist();
  }


  @Override
  protected void interrupted() {
      return;
  }
}

