package frc.robot.commands.drive.outtake; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends Command{
    private IntakeSubsystem intake;

    public OuttakeCommand(){
        requires(Robot.intake);
        intake = Robot.intake;
    }

    @Override
    protected void initialize() {
        intake.stop();
    }


  @Override
  protected void execute() {
      Robot.intake.stop();
  }


  @Override
  protected boolean isFinished() {
    return Robot.m_oi.getController1().getRawButtonReleased(XboxControllerButtonCode.RB.getCode());
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

