package frc.robot.commands.intake; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.input.XboxControllerButtonCode;

public class RotateWristUpCommand extends Command{

    public RotateWristUpCommand(){
        requires(Robot.intake);
    }

    @Override
    protected void initialize() {
        Robot.intake.setAngle(Robot.intake.WristMaxDegrees);
    }

  @Override
  protected void execute() {
    //initialize does all the work
  }

  @Override
  protected boolean isFinished() { return true;  }
 
  @Override
  protected void end() {  }

  @Override
  protected void interrupted() {  }
}

