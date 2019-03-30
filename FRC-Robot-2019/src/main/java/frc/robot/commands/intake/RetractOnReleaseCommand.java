package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.CommandManager;

public class RetractOnReleaseCommand extends Command {
  BooleanSupplier releaseCheckFunc;
  double x_retract;
  double timeout;
  CommandManager cmdMgr;
  double init_x;
  double init_h;

  public RetractOnReleaseCommand(CommandManager cmdMgr, double x_retract, double timeout) {
    //could require(vacSensor0)
    this.releaseCheckFunc = Robot.intake.getVacuumSensor()::hasReleased;
    this.x_retract = x_retract;
    this.timeout = timeout;
    this.cmdMgr = cmdMgr;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //save were we are so we can tweek it on finish
    init_x = cmdMgr.gripperXProjectionOut();
    init_h = cmdMgr.gripperHeightOut();
    setTimeout(timeout);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   //DEBUG CODE
    boolean rc= releaseCheckFunc.getAsBoolean();
    SmartDashboard.putBoolean("ReleaseSensor", rc);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut() || releaseCheckFunc.getAsBoolean();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // we are done, we timed out or we got the vacuum release signal,  move us back.
    double x = init_x;
    x -= Robot.arm.getInversion()*x_retract;   //move back a bit, account for side.
    cmdMgr.cmdPosition(init_h, x);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
