package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class TankDriveCommand extends CommandBase {
  private DriveTrainSubsystem driveTrain = Robot.driveTrain;
  private XboxController ctrl;

  public TankDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(Robot.driveTrain);
  }
  
  // Called just before this Command runs the first time
  @Override
 public void initialize() {
    driveTrain.stop();
    ctrl = Robot.m_oi.getDriverController();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  public void execute() {
    Robot.driveTrain.tankDrive(ctrl.getY(Hand.kLeft), ctrl.getY(Hand.kRight), true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

}