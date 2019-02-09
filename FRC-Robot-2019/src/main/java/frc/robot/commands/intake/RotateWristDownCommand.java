package frc.robot.commands.intake; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;

public class RotateWristDownCommand extends Command{

    public RotateWristDownCommand(){
        requires(Robot.intake);
    }

    @Override
    protected void initialize() {
        Robot.intake.stopWrist();
    }


  @Override
  protected void execute() {
      Robot.intake.runWristDown();

  }

  @Override
  protected boolean isFinished() {
    return Robot.m_oi.getController1().getRawButtonReleased(XboxControllerButtonCode.BACK.getCode());
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

