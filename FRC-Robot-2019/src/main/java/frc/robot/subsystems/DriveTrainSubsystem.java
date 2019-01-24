package frc.robot.subsystems;

import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.TankDriveCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * The basic drive train subsystem for four motors
 */
public class DriveTrainSubsystem extends Subsystem {

  // Individual Motors
  private WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(RobotMap.FL_TALON_CAN_ID);
  private WPI_TalonSRX middleLeftMotor = new WPI_TalonSRX(RobotMap.ML_TALON_CAN_ID);
  private WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(RobotMap.BL_TALON_CAN_ID);
  private WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(RobotMap.FR_TALON_CAN_ID);
  private WPI_TalonSRX middleRightMotor = new WPI_TalonSRX(RobotMap.MR_TALON_CAN_ID);
  private WPI_TalonSRX backRightMotor = new WPI_TalonSRX(RobotMap.BR_TALON_CAN_ID);
  private Encoder leftEncoder = new Encoder(RobotMap.ENCODER_LEFT_PIN_1, RobotMap.ENCODER_LEFT_PIN_2, false, EncodingType.k4X);
  private Encoder rightEncoder = new Encoder(RobotMap.ENCODER_RIGHT_PIN_1, RobotMap.ENCODER_RIGHT_PIN_2, false, EncodingType.k4X);

  // Motor groups
  private SpeedControllerGroup leftMotors, rightMotors;

  private DifferentialDrive drive;

  public DriveTrainSubsystem() {

    addChild("Front Left CIM", (Sendable) frontLeftMotor);
    addChild("Middle Left CIM", (Sendable) middleLeftMotor);
    addChild("Back Left CIM", (Sendable) backLeftMotor);
    addChild("Front Right CIM", (Sendable) frontRightMotor);
    addChild("Middle Right CIM", (Sendable) middleRightMotor);
    addChild("Back Right CIM", (Sendable) backRightMotor);
    addChild("Left Encoder", (Sendable) leftEncoder);
    addChild("Right Encoder", (Sendable) rightEncoder);

    leftMotors = new SpeedControllerGroup(frontLeftMotor, middleLeftMotor, backLeftMotor);
    rightMotors = new SpeedControllerGroup(frontRightMotor, middleRightMotor, backRightMotor);

    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArcadeDriveCommand());
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
   * Arcade drive using a single joystick
   *
   * @param xSpeed        The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation     The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                      positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void ArcadeDrive(double xSpeed, double zRotation, boolean squaredInput) {
    drive.arcadeDrive(xSpeed, zRotation, squaredInput);
  }

  public void driveDistance(int distance) {
    frontLeftMotor.follow(frontRightMotor, FollowerType.AuxOutput1);
    frontRightMotor.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, 0);
    
  }

  /**
   * Get the left encoder
   * @return The left encoder object
   */
  public Encoder getLeftEncoder(){
    return leftEncoder;
  }

  /**
   * Get the right encoder
   * @return The right encoder object
   */
  public Encoder getRightEncoderRate(){
    return rightEncoder;
  }

  /**
   * Stop the drivetrain from moving.
   */
  public void stop() {
    drive.stopMotor();
  }
}
