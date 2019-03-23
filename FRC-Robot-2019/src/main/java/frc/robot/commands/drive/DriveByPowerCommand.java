package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveByPowerCommand extends Command {
  private DriveTrainSubsystem driveTrain = Robot.driveTrain;
  double power;
  double timeout;

  public DriveByPowerCommand(double power, double timeout) {
    this.power = power;
    this.timeout = timeout;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //  driveTrain.stop();
    // may want to check counters... if we try to control this...
    setTimeout(timeout);
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  protected void execute() {
    Robot.driveTrain.tankDrive(power, power, true);
  }

  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  @Override
  protected void end() {
    driveTrain.stop();
  }

  @Override
  protected void interrupted() {
  }
}