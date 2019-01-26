package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class ArcadeDriveCommand extends Command {
  private DriveTrainSubsystem driveTrain;

  public ArcadeDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    driveTrain = Robot.driveTrain;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.stop();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  protected void execute() {
    //Robot.driveTrain.ArcadeDrive(0.90, 0, true);
    Robot.driveTrain.ArcadeDrive(Robot.m_oi.getController0().getY(Hand.kLeft), -Robot.m_oi.getController0().getX(Hand.kLeft), true);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    driveTrain.stop();
  }
}