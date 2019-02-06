package frc.robot.commands; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateWristCommand extends Command{
    private IntakeSubsystem intake;

    public RotateWristCommand(){
        requires(Robot.intake);
        intake = Robot.intake;
    }

    @Override
    protected void initialize() {
        intake.stop();
    }


  @Override
  protected void execute() {
      Robot.intake.run();
      double placeHolderValue = .5;
      Robot.intake.setWristPosition(placeHolderValue);

  }


  @Override
  protected boolean isFinished() {
    return intake.getCargoSwitch() || Robot.m_oi.getController1().getRawButtonReleased(XboxControllerButtonCode.LB.getCode());
  }

 
  @Override
  protected void end() {
      intake.stop();
  }


  @Override
  protected void interrupted() {
      return;
  }
}

