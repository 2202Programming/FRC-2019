package frc.robot.commands.intake; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RotateWristCommand extends Command{
    private double angle;

    public RotateWristCommand(double angle){
        requires(Robot.intake);
        this.angle = angle;
    }

    @Override
    protected void initialize() {
        Robot.intake.stop();
    }


  @Override
  protected void execute() {
      Robot.intake.setWristAngle(angle);

  }


  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.intake.getWristAngle() - angle) < 1;
  }

 
  @Override
  protected void end() {
      Robot.intake.stop();
  }


  @Override
  protected void interrupted() {
      return;
  }
}

