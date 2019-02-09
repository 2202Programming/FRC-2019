package frc.robot.commands; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    private IntakeSubsystem intake;

    public IntakeCommand(){
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

