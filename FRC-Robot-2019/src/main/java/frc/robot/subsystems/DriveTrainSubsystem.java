package frc.robot.subsystems;

import frc.robot.commands.TankDriveCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.RobotMap;

/**
 * The basic drive train subsystem for four motors
 */
public class DriveTrainSubsystem extends Subsystem {

  // Individual Motors
  private SpeedController frontLeftMotor = new Spark(RobotMap.FL_MOTOR_PIN);
  private SpeedController backLeftMotor = new Spark(RobotMap.BL_MOTOR_PIN);
	private SpeedController frontRightMotor = new Spark(RobotMap.FR_MOTOR_PIN);
  private SpeedController backRightMotor = new Spark(RobotMap.BR_MOTOR_PIN);
  // Motor groups
  private SpeedControllerGroup leftMotors, rightMotors;

  private DifferentialDrive drive;

  public DriveTrainSubsystem() {

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
