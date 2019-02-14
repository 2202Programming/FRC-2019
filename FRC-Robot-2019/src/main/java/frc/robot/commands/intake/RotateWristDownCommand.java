package frc.robot.commands.intake; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.input.XboxControllerButtonCode;
import frc.robot.subsystems.IntakeSubsystem;


/**
 * Simple command that rotates wrist down to fixed spot for testing
 */
public class RotateWristDownCommand extends Command{

    IntakeSubsystem intake = Robot.intake;

    public RotateWristDownCommand(){
        requires(intake);
    }

    @Override
    protected void initialize() {
        Robot.intake.setAngle(intake.WristMinDegrees);
    }

  @Override
  protected void execute() { }
      

  @Override
  protected boolean isFinished() {
      // Servo will just goto the command, could do a wait in the exec??? ###
    return true;
  }
 
  @Override
  protected void end() {  } 

  @Override
  protected void interrupted() {  }
}

