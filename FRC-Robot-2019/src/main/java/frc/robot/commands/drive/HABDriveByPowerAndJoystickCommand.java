package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.util.ExpoShaper;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class HABDriveByPowerAndJoystickCommand extends Command {
  private DriveTrainSubsystem driveTrain = Robot.driveTrain;
  double power;
  double minPower;
  double maxPower;
  double timeout;
  private ExpoShaper speedShaper;
  private ExpoShaper rotationShaper;

  public HABDriveByPowerAndJoystickCommand(double power, double minPower, double maxPower) {
    this.power = power;
    this.minPower = minPower;
    this.maxPower = maxPower;
    this.speedShaper = new ExpoShaper(0.6);        //0 no change,  1.0 max flatness
    this.rotationShaper = new ExpoShaper(0.5);
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }
  
  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  protected void execute() {
    double speedInput = speedShaper.expo(Robot.m_oi.getDriverController().getY(Hand.kLeft));
    double speedAdjust = speedInput > 0? speedInput * (maxPower - power): speedInput * (power - minPower);
    double rotation = 0.5 * rotationShaper.expo(Robot.m_oi.getDriverController().getX(Hand.kRight));
    Robot.driveTrain.ArcadeDrive(power + speedAdjust, rotation, true);
  }

  @Override
  protected boolean isFinished() {
    return Robot.climber.climberAgainstWall();
  }

  @Override
  protected void end() {
    driveTrain.stop();
  }

  @Override
  protected void interrupted() {
  }
}