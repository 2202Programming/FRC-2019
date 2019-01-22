package frc.robot.commands; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    private IntakeSubsystem intake;

    public IntakeCommand(){
        requires(Robot.m_subsystem);
        intake = Robot.intake;
    }

    @Override
    protected void initialize() {
        intake.stop();
    }


  @Override
  protected void execute() {
      Robot.intake.intake();
  }


  @Override
  protected boolean isFinished() {
    return intake.getPhotoGate() || Robot.m_oi.getController0().getRawButtonReleased(5);
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

