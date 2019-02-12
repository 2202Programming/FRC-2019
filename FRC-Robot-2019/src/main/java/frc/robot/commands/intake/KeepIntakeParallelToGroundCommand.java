package frc.robot.commands.intake; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;
import frc.robot.subsystems.IntakeSubsystem;

public class KeepIntakeParallelToGroundCommand extends Command{
    private double parallelAngle;

    public KeepIntakeParallelToGroundCommand(){
        requires(Robot.intake);
        parallelAngle = 90 - Robot.arm.getAngle();
    }

    @Override
    protected void execute() {
        Robot.intake.setWristAngle(parallelAngle);
    }


  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.intake.getWristAngle() - parallelAngle) < 1;
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

