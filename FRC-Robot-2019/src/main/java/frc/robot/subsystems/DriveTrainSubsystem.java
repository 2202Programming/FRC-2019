package frc.robot.subsystems;

import frc.robot.commands.TankDriveCommand;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * The basic drive train subsystem for four motors
 */
public class DriveTrainSubsystem extends Subsystem {

  // Individual Motors
  private SpeedController frontLeftMotor = new WPI_TalonSRX(RobotMap.FL_TALON_CAN_ID);
  private SpeedController middleLeftMotor = new WPI_TalonSRX(RobotMap.ML_TALON_CAN_ID);
  private SpeedController backLeftMotor = new WPI_TalonSRX(RobotMap.BL_TALON_CAN_ID);
  private SpeedController frontRightMotor = new WPI_TalonSRX(RobotMap.FR_TALON_CAN_ID);
  private SpeedController middleRightMotor = new WPI_TalonSRX(RobotMap.MR_TALON_CAN_ID);
  private SpeedController backRightMotor = new WPI_TalonSRX(RobotMap.BR_TALON_CAN_ID);
  // Motor groups
  private SpeedControllerGroup leftMotors, rightMotors;

  private DifferentialDrive drive;

  public DriveTrainSubsystem() {

    addChild("Front Left CIM", (Sendable) frontLeftMotor);
    addChild("Back Left CIM", (Sendable) backLeftMotor);
    addChild("Front Right CIM", (Sendable) frontRightMotor);
    addChild("Back Right CIM", (Sendable) backRightMotor);

    leftMotors = new SpeedControllerGroup(frontLeftMotor, middleLeftMotor, backLeftMotor);
    rightMotors = new SpeedControllerGroup(frontRightMotor, middleRightMotor, backRightMotor);

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
