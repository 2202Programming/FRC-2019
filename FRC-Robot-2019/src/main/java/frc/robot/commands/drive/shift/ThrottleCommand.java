package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * An example command. You can replace me with your own command.
 */
public class ThrottleCommand extends CommandBase {
  private final double CYCLE_TIME_IN_SECONDS = 0.020;
  private int cycleCount;
  private int maxCycles;
  private double stepValue;
  private double startValue;

  /**
   * 
   * @param rampTime Ramp up time in seconds
   */
  public ThrottleCommand(double rampTime, double startValue, double endValue) {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(Robot.driveTrain);
    maxCycles = (int) Math.ceil(rampTime / CYCLE_TIME_IN_SECONDS);
    this.startValue = startValue;
    stepValue = (endValue - startValue) / maxCycles;
  }

  // Called just before this Command runs the first time
  @Override
 public void initialize() {
    cycleCount = 0;
    execute();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  @Override
  public void execute() {
    double throttle = Robot.m_oi.getDriverController().getLeftY() * (startValue + stepValue * cycleCount);
    double turnRate = Robot.m_oi.getDriverController().getRightX() * (startValue + stepValue * cycleCount);
    cycleCount++;
    Robot.driveTrain.ArcadeDrive(throttle, turnRate, true);
  }

  @Override
  public boolean isFinished() {
    double leftSpeed = Math.abs(Robot.driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity());
    double rightSpeed = Math.abs(Robot.driveTrain.getRightEncoderTalon().getSelectedSensorVelocity());
    double curSpeed = (leftSpeed + rightSpeed) / 2.0;
    double shiftSpeed = AutomaticGearShiftCommand.DOWNSHIFT_SPEED_LOW * AutomaticGearShiftCommand.MAXSPEED_IN_COUNTS_PER_SECOND;
    return cycleCount >= maxCycles || curSpeed < shiftSpeed;
  }

  @Override
  public void end(boolean interrupted) {
  }
}