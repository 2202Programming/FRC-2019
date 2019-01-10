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

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  protected void execute() {
		Robot.driveTrain.tankDrive(Robot.m_oi.getMainJoystick().getRawAxis(1), Robot.m_oi.getMainJoystick().getRawAxis(5));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    driveTrain.stop();
  }

  @Override
  protected void interrupted() {
    return'
  }
}