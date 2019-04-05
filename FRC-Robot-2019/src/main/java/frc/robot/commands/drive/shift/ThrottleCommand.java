package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class ThrottleCommand extends Command {
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
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    maxCycles = (int) Math.ceil(rampTime / CYCLE_TIME_IN_SECONDS);
    this.startValue = startValue;
    stepValue = (endValue - startValue) / maxCycles;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    cycleCount = 0;
    execute();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  @Override
  protected void execute() {
    double throttle = Robot.m_oi.getDriverController().getY(Hand.kLeft) * (startValue + stepValue * cycleCount);
    double turnRate = Robot.m_oi.getDriverController().getX(Hand.kRight) * (startValue + stepValue * cycleCount);
    cycleCount++;
    Robot.driveTrain.ArcadeDrive(throttle, turnRate, true);
  }

  @Override
  protected boolean isFinished() {
    double leftSpeed = Math.abs(driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity());
    double rightSpeed = Math.abs(driveTrain.getRightEncoderTalon().getSelectedSensorVelocity());
    double curSpeed = (leftSpeed + rightSpeed) / 2.0;
    double shiftSpeed = AutomaticGearShiftCommand.DOWNSHIFT_SPEED_LOW * AutomaticGearShiftCommand.MAXSPEED_IN_COUNTS_PER_SECOND;
    return cycleCount >= maxCycles || curSpeed < shiftSpeed;
  }

  @Override
  protected void end() {
  }
}