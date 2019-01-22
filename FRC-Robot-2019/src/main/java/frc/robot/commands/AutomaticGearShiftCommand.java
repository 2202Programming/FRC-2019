package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Automatically shifts the 
 */
public class AutomaticGearShiftCommand extends Command {
  private final double UPSHIFT_SPEED_LOW = 0.4;
  private final double UPSHIFT_SPEED_HIGH = 0.4;
  private final double UPSHIFT_THROTTLE_LOW = 0.3;
  private final double UPSHIFT_THROTTLE_HIGH = 0.6;
  private final double DOWNSSHIFT_SPEED_LOW = 0.1;
  private final double DOWNSSHIFT_SPEED_HIGH = 0.1;
  private final double DOWNSHIFT_THROTTLE_LOW = 0.3;
  private final double DOWNSHIFT_THROTTLE_HIGH = 0.6;
  private final double DEADZONE = 0.02; 
  private final double MAX_OUTPUT = 1.0;
  private final double RIGHT_SIDE_INVERT_MULTIPLIER = -1.0;
  private double leftThrottle;
  private double rightThrottle;

  public AutomaticGearShiftCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.gearShifter);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  /**
   * Taken from DifferentialDrive arcadeDrive method
   */
  private void setThrottles(boolean squareInputs) {
    double xSpeed = limit(Robot.m_oi.getController0().getLeftJoystickY());
    xSpeed = applyDeadband(xSpeed, DEADZONE);

    double zRotation = limit(Robot.m_oi.getController0().getLeftJoystickX());
    zRotation = applyDeadband(zRotation, DEADZONE);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    leftThrottle = limit(leftMotorOutput) * MAX_OUTPUT;
    rightThrottle = limit(rightMotorOutput) * MAX_OUTPUT * RIGHT_SIDE_INVERT_MULTIPLIER;
  }

    /**
   * Limit motor values to the -1.0 to +1.0 range.
   */
  private double limit(double value) {
    if (value > 1.0) {
      return 1.0;
    }
    if (value < -1.0) {
      return -1.0;
    }
    return value;
  }

  /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value    value to clip
   * @param deadband range around zero
   */
  protected double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
}