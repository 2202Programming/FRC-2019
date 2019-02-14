package frc.robot.commands.intake; 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateWristCommand extends Command{
    IntakeSubsystem intake = Robot.intake;
    XboxController ctrl = Robot.m_oi.getAssistantController();

    public RotateWristCommand(){
        requires(Robot.intake);
    }

    @Override
    protected void initialize() {
        intake.vacuumOn();
    }

  @Override
  protected void execute() {
      double cmd = ctrl.getY(Hand.kRight);
      double degrees = cmd * intake.WristMaxDegrees;  //### assumes symetric up/down
      intake.setAngle(degrees);
  }


  @Override
  protected boolean isFinished() {
    //return intake.getCargoSwitch() || ctrl.getRawButtonReleased(XboxControllerButtonCode.LB.getCode());
    // keep doing this
    return false;
  }

  @Override
  protected void end() {
      // DPL - I think we just keep the vacuum on until some some other command takes place
  }


  @Override
  protected void interrupted() {
      return;
  }
}

