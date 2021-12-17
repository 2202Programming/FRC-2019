package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.drive.ArcadeDriveCommand;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;

/**
 * The basic drive train subsystem for four motors
 */
public class DriveTrainSubsystem extends SubsystemBase {

  //values in inches 
  public final double ENCODER_RIGHT_DISTANCE_PER_PULSE = 0.002396219298;
  public final double ENCODER_LEFT_DISTANCE_PER_PULSE = 0.002399087014;
  public final int ENCODER_COUNTS_PER_REVOLUTION = 8192; //Double check this
  public final double WHEEL_RADIUS = 3;
 
  public final double kSamplePeriod = 0.1;  // 100mS in Talon

  // Individual Motors
  private WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(RobotMap.FL_TALON_CAN_ID);
  private WPI_TalonSRX middleLeftMotor = new WPI_TalonSRX(RobotMap.ML_TALON_CAN_ID);
  private WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(RobotMap.BL_TALON_CAN_ID);
  private WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(RobotMap.FR_TALON_CAN_ID);
  private WPI_TalonSRX middleRightMotor = new WPI_TalonSRX(RobotMap.MR_TALON_CAN_ID);
  private WPI_TalonSRX backRightMotor = new WPI_TalonSRX(RobotMap.BR_TALON_CAN_ID);
  private SpeedController leftMotors;
  private SpeedController rightMotors;
  private WPI_TalonSRX leftEncoder;
  private WPI_TalonSRX rightEncoder;

  //GearShifter 
  public final double kShiftPoint = 3000.0;  // Encoder velocity; Maximum shift down velocity

  private DifferentialDrive drive;

  private short inversionConstant;

  private final double DEADZONE = 0.02; 
  private final double MAX_OUTPUT = 1.0;
  private final double RIGHT_SIDE_INVERT_MULTIPLIER = -1.0;

  private NetworkTableEntry cameraSelect;
  private long logTimer;

  public DriveTrainSubsystem() {    
    addChild("Middle Left CIM", (Sendable) middleLeftMotor);
    addChild("Back Left CIM", (Sendable) backLeftMotor);
    addChild("Front Right CIM", (Sendable) frontRightMotor);
    addChild("Middle Right CIM", (Sendable) middleRightMotor);
    addChild("Back Right CIM", (Sendable) backRightMotor);
    addChild("Front Left CIM", (Sendable) frontLeftMotor);
    
    leftEncoder = (WPI_TalonSRX) frontLeftMotor;
    leftEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    rightEncoder = (WPI_TalonSRX) frontRightMotor;

    rightEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    rightEncoder.setSensorPhase(true);

    middleLeftMotor.follow(frontLeftMotor);
    backLeftMotor.follow(frontLeftMotor);

    middleRightMotor.follow(frontRightMotor);
    backRightMotor.follow(frontRightMotor);

    limitTalon(frontLeftMotor);
    limitTalon(middleLeftMotor);
    limitTalon(backLeftMotor);
    limitTalon(frontRightMotor);
    limitTalon(middleRightMotor);
    limitTalon(backRightMotor);
    
    leftMotors = frontLeftMotor;
    rightMotors = frontRightMotor;

    drive = new DifferentialDrive(leftMotors, rightMotors);
    inversionConstant = 1;
    
    cameraSelect = NetworkTableInstance.getDefault().getEntry("/PiSwitch");
    logTimer = System.currentTimeMillis();
  }

  public void log(int interval) {
    if ((logTimer + interval) < System.currentTimeMillis()) { //only post to smartdashboard every interval ms
      logTimer = System.currentTimeMillis();

      SmartDashboard.putData(this);
      SmartDashboard.putNumber("Left Encoder Count", getLeftEncoderTalon().getSelectedSensorPosition());
      SmartDashboard.putNumber("Left Encoder Rate", getLeftEncoderTalon().getSelectedSensorVelocity());
      SmartDashboard.putNumber("Left Motor Output", getLeftEncoderTalon().getMotorOutputPercent());
      SmartDashboard.putNumber("Right Encoder Count", getRightEncoderTalon().getSelectedSensorPosition());
      SmartDashboard.putNumber("Right Encoder Rate", getRightEncoderTalon().getSelectedSensorVelocity());
      SmartDashboard.putNumber("Right Motor Output", getRightEncoderTalon().getMotorOutputPercent());
    }
  }

  private void limitTalon(WPI_TalonSRX talon){
    talon.configPeakCurrentLimit(0, 10);
    talon.configPeakCurrentDuration(0, 10);
    talon.configContinuousCurrentLimit(30, 10);
    talon.enableCurrentLimit(true);
    talon.configOpenloopRamp(0.08, 10);
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configPeakOutputForward(0.95);
    talon.configPeakOutputReverse(-0.95);
  }

  // TODO - WIRE THIS UP -DPL  12/16/2021
  public void initDefaultCommand() {
    leftEncoder.setSelectedSensorPosition(0);
    rightEncoder.setSelectedSensorPosition(0);
    inversionConstant = 1;
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
  public void tankDrive(double leftAxis, double rightAxis, boolean squareInputs) {
    drive.tankDrive(inversionConstant * leftAxis, inversionConstant * rightAxis, squareInputs);
    logTankThrottles(leftAxis, rightAxis, squareInputs);
  }

  /**
   * Arcade drive using a single joystick
   *
   * @param xSpeed        The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation     The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                      positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void ArcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    drive.arcadeDrive(inversionConstant * xSpeed, -zRotation, squareInputs);
    logArcadeThrottles(xSpeed, zRotation, squareInputs);
  }

  /**
   * Get the left encoder
   * @return The left encoder object
   */
  public TalonSRX getLeftEncoderTalon(){
    return leftEncoder;
  }

  /**
   * Get the right encoder
   * @return The right encoder object
   */
  public TalonSRX getRightEncoderTalon(){
    return rightEncoder;
  }

  /**
   * Reset the left encoder position
   */
  public void resetLeftEncoderTalon(){
    leftEncoder.setSelectedSensorPosition(0);
  }

  /**
   * Reset the right encoder position
   */
  public void resetRightEncoderTalon(){
    rightEncoder.setSelectedSensorPosition(0);
  }

  /**
   * Stop the drivetrain from moving.
   */
  public void stop() {
    drive.stopMotor();
  }

  /**
   * Inverts the driver controls
   */
  public void invertControls() {
    inversionConstant *= -1;
    Robot.cameraSubsystem.toggleDriveCamera();
    
    //post to network tables which drive camera to show based on control direction
    if (inversionConstant>0) {
      cameraSelect.setDouble(0);
    }
    else {
      cameraSelect.setDouble(1);
    }
  }

   /**
   * Inverts the driver controls
   */
  public int getInversionConstant() {
    return inversionConstant;
  }
  public double posLeft() {
    //pos = encoderCount * dist/counts 
    return leftEncoder.getSelectedSensorPosition() * ENCODER_LEFT_DISTANCE_PER_PULSE;
  }
  public double posRight() {
    return rightEncoder.getSelectedSensorPosition() * ENCODER_RIGHT_DISTANCE_PER_PULSE;
  }
  public double velLeft() {
    //lin vel = counts/sampletime * sampletime/sec * dist/counts
    //0.1 is assuming that the sample time is 100ms
    return (leftEncoder.getSelectedSensorVelocity() * kSamplePeriod  * ENCODER_LEFT_DISTANCE_PER_PULSE);
  }
  public double velRight() {
    return (rightEncoder.getSelectedSensorVelocity()* kSamplePeriod * ENCODER_RIGHT_DISTANCE_PER_PULSE);
  }

  /**
   * Deadband in inches/sec
   */
  public boolean isMoving(double deadband) {
    return Math.abs(velLeft()) < deadband || Math.abs(velRight()) < deadband;
  }

  /**
   * Taken from DifferentialDrive tankDrive method
   * Tank drive method for differential drive platform.
   *
   * @param leftSpeed     The robot left side's speed along the X axis [-1.0..1.0]. Forward is
   *                      positive.
   * @param rightSpeed    The robot right side's speed along the X axis [-1.0..1.0]. Forward is
   *                      positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  private void logTankThrottles(double leftSpeed, double rightSpeed, boolean squareInputs) {
    leftSpeed = limit(leftSpeed);
    leftSpeed = applyDeadband(leftSpeed, DEADZONE);

    rightSpeed = limit(rightSpeed);
    rightSpeed = applyDeadband(rightSpeed, DEADZONE);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed);
      rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed);
    }

    double leftThrottle = leftSpeed * MAX_OUTPUT;
    double rightThrottle = rightSpeed * MAX_OUTPUT * RIGHT_SIDE_INVERT_MULTIPLIER;

    SmartDashboard.putNumber("Left Throttle", leftThrottle);
    SmartDashboard.putNumber("Right Throttle", rightThrottle);
  }

  /**
   * Taken from DifferentialDrive arcadeDrive method
   * Gets the minimum throttle between the two sides of the robot
   * 
   * @param xSpeed        The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation     The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                      positive.
   * @param squareInputs Whether you are using squared inputs
   */
  private void logArcadeThrottles(double xSpeed, double zRotation, boolean squareInputs) {
    xSpeed = applyDeadband(xSpeed, DEADZONE);

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

    double leftThrottle = Math.abs(limit(leftMotorOutput) * MAX_OUTPUT);
    double rightThrottle = Math.abs(limit(rightMotorOutput) * MAX_OUTPUT * RIGHT_SIDE_INVERT_MULTIPLIER);
    SmartDashboard.putNumber("Left Throttle", leftThrottle);
    SmartDashboard.putNumber("Right Throttle", rightThrottle);
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
  private double applyDeadband(double value, double deadband) {
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

  /**
   * Logs all of the WPI_TalonSRX instance fields.
   */
  public void logTalons() {
    logTalon(frontLeftMotor);
    logTalon(frontRightMotor);
    logTalon(middleLeftMotor);
    logTalon(middleRightMotor);
    logTalon(backLeftMotor);
    logTalon(backRightMotor);
    logTalon(leftEncoder);
    logTalon(rightEncoder);
  }

  /**
   * Logs a WPI_TalonSRX.
   * @param talon the talon to be logged.
   */
  public void logTalon(WPI_TalonSRX talon) {
    SmartDashboard.putNumber(talon.getName() + " Current", talon.getSupplyCurrent());
  }
}
