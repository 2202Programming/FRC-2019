package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class TankDriveCommand extends Command {
  private DriveTrainSubsystem driveTrain = Robot.driveTrain;
  private XboxController ctrl;

  public TankDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.stop();
    ctrl = Robot.m_oi.getDriverController();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  protected void execute() {
    Robot.driveTrain.tankDrive(ctrl.getY(Hand.kLeft), ctrl.getY(Hand.kRight), true);
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
    return;
  }
}