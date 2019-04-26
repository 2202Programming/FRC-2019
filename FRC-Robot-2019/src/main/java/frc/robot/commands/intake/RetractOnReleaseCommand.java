package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.CommandManager;
import frc.robot.commands.arm.ArmStatePositioner;

public class RetractOnReleaseCommand extends Command {
  BooleanSupplier releaseCheckFunc;
  double x_retract;
  double timeout;
  double init_x;
  double init_h;
  double x_new;    //x to jump to when the sensor says we are released

  private ArmStatePositioner armPositioner;

  public RetractOnReleaseCommand(CommandManager cmdMgr, double x_retract, double timeout) {
    //could require(vacSensor0)
    this.releaseCheckFunc = Robot.intake.getVacuumSensor()::hasReleased;
    this.x_retract = x_retract;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Assume that the ArmStatePositioner is the only type of default command used
    armPositioner = Robot.arm.getArmPositioner();;
    //save were we are so we can tweek it on finish
    init_x = armPositioner.getProjectionCommanded();
    init_h = armPositioner.getHeightCommanded();
    x_new = init_x;
    int invertMultiplier = Robot.arm.isInverted()? -1 : 1;
    x_new -= invertMultiplier * x_retract;   //move back a bit, account for side.
    
    setTimeout(timeout);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   //DEBUG CODE
    boolean rc= releaseCheckFunc.getAsBoolean();
    SmartDashboard.putBoolean("ReleaseSensor", rc);
    // we are done, we timed out or we got the vacuum release signal,  move us back.
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut() || releaseCheckFunc.getAsBoolean();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //We are released according to the sensor, NOW move the arm, if we didnt timeout.
    if (!isTimedOut()) {
       armPositioner.setPosition(init_h, x_new); 
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
