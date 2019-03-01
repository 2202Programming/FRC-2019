package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.commands.util.ExpoShaper;
/**
 * An example command. You can replace me with your own command.
 */
public class ArcadeDriveCommand extends Command {
  private DriveTrainSubsystem driveTrain;
  private ExpoShaper speedShaper;
  private ExpoShaper rotationShaper;

  public ArcadeDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    driveTrain = Robot.driveTrain;

    speedShaper = new ExpoShaper(0.5);        //0 no change,  1.0 max flatness
    rotationShaper = new ExpoShaper(0.5);
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
    double s = speedShaper.expo(Robot.m_oi.getDriverController().getY(Hand.kLeft));
    double rot = rotationShaper.expo(Robot.m_oi.getDriverController().getX(Hand.kRight));
    Robot.driveTrain.ArcadeDrive(s, rot, true);
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