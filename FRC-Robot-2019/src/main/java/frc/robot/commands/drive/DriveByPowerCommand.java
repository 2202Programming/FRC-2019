package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveByPowerCommand extends WaitCommand {
  private DriveTrainSubsystem driveTrain = Robot.driveTrain;
  double power;
  double timeout;

  public DriveByPowerCommand(double power, double timeout) {
    super(timeout);
    this.power = power;
    this.timeout = timeout;
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(Robot.driveTrain);
  }
  

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  public void execute() {
    Robot.driveTrain.ArcadeDrive(power, 0.0, true);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    driveTrain.stop();
  }

}