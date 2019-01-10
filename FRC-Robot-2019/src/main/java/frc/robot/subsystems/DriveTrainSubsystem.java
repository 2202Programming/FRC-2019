package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.commands.TankDriveCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The basic drive train subsystem for four motors
 */
public class DriveTrainSubsystem extends Subsystem {

  // Individual Motors
  private SpeedController frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

  // Motor groups
  private SpeedControllerGroup leftMotors, rightMotors;

  private DifferentialDrive drive;

  public DriveTrainSubsystem() {
    frontLeftMotor = Robot.m_oi.getFrontLeft();
    backLeftMotor = Robot.m_oi.getBackLeft();
    frontRightMotor = Robot.m_oi.getFrontRight();
    backRightMotor = Robot.m_oi.getBackRight();

    addChild("Front Left CIM", (Sendable) frontLeftMotor);
    addChild("Back Left CIM", (Sendable) backLeftMotor);
    addChild("Front Right CIM", (Sendable) frontRightMotor);
    addChild("Back Right CIM", (Sendable) backRightMotor);

    leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);

    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDriveCommand());
  }

  /**
   * Tank drive using a PS3 joystick.
   *
   * @param joy PS3 style joystick to use as the input for tank drive.
   */
  public void tankDrive(Joystick joy) {
    drive.tankDrive(joy.getY(), joy.getRawAxis(4));
  }

  /**
   * Tank drive using individual joystick axes.
   *
   * @param leftAxis  Left sides value
   * @param rightAxis Right sides value
   */
  public void tankDrive(double leftAxis, double rightAxis) {
    drive.tankDrive(leftAxis, rightAxis);
  }

  /**
   * Stop the drivetrain from moving.
   */
  public void stop() {
    drive.stopMotor();
  }
}
