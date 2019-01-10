package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command.  You can replace me with your own command.
 */
public class TankDriveCommand extends Command {
  private DriveTrainSubsystem driveTrain;

  public TankDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_subsystem);
    driveTrain = Robot.driveTrain;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
		Robot.driveTrain.tankDrive(Robot.m_oi.getMainJoystick());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}