package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class ThrottleCommand extends Command {
  private final double CYCLE_TIME_IN_SECONDS = 0.020;
  private DriveTrainSubsystem driveTrain;
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
    driveTrain = Robot.driveTrain;
    cycleCount = 0;
    maxCycles = (int) Math.ceil(rampTime / CYCLE_TIME_IN_SECONDS);
    this.startValue = startValue;
    stepValue = (endValue - startValue) / maxCycles;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.stop();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  @Override
  protected void execute() {
    double throttle = Robot.m_oi.getController0().getY(Hand.kLeft) * (startValue + stepValue * cycleCount);
    cycleCount++;
    double turnRate = Robot.m_oi.getController0().getX(Hand.kLeft) * (startValue + stepValue * cycleCount);
    Robot.driveTrain.ArcadeDrive(throttle, turnRate, true);
  }

  @Override
  protected boolean isFinished() {
    return cycleCount >= maxCycles;
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