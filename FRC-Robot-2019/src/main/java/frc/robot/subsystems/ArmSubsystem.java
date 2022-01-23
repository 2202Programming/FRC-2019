package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.arm.ArmStatePositioner;
import frc.robot.commands.arm.ArmZero;
import frc.robot.commands.util.MathUtil;

/**
 * Arm based lifter subsystem.
 * 
 * PHI0 and L0 are the starting positons where encoders are zeroed. Happens at
 * teleOp init. So we expect to start system off at known location.
 * 
 * 2/20/19 DPL updated constants from measured protobot removed encoder zeros
 * from init, leave that to zeroArm moved logging from robot to here, removed
 * public accesss to encoders double checked math with non-zero initial start
 * positons
 * 
 * 
 * Arm gearing M:7:E:7:3:shaft:(12->30) where M=motor, 7= 7:1, 3= 3:1 ( 12->30 )
 * pulley teeth (2.5) E: encoder 1024 counts/rev quad encoder
 * 
 * ---->> 149.3333 counts/(deg arm) WTH? measured ~600
 * 
 * Ext gearing M:5:7:E:shaft:(42:22:36):5mm pitch 36 teeth@5mm
 * 
 * 
 * 
 */
public class ArmSubsystem extends SubsystemBase {
  private WPI_TalonSRX armRotationMotor = new WPI_TalonSRX(RobotMap.ARM_ROTATION_TALON_CAN_ID);
  private WPI_TalonSRX armExtensionMotor = new WPI_TalonSRX(RobotMap.ARM_EXTENSTION_TALON_CAN_ID);
  private DigitalInput extensionAtMin = new DigitalInput(RobotMap.ARM_MIN_EXTENSION_SENSOR_PIN);

  // Constants used by commands as measured
  // When on the ground we can't touch the hard stop. We are off by ~1 degree
  public final double PHI0 = 158.0; // degrees, starting position - encoder zero
  public final double PHI_MAX = 158.0; // In Degrees, Positive is foward, bottom front
  public final double PHI_FRONT_MIN = 25.0; // In Degrees, Near top front
  public final double PHI_BACK_MAX = -25.0; // In degrees
  public final double PHI_MIN = -140.0; // In degress

  private final double kCounts_per_deg = 843; // back to practice bot
  private final double kDeg_per_count = 1.0 / kCounts_per_deg;

  // Geometry of the arm's pivot point
  public final double PIVOT_TO_FRONT = 16.5; // inches pivot center to the frame
  public final double MIN_FRONT_PROJECTION = PIVOT_TO_FRONT - 6.5; // inches from pivot to close arm position
  public final double MIN_BACK_PROJECTION = -MIN_FRONT_PROJECTION;
  public final double MAX_PROJECTION = PIVOT_TO_FRONT + Robot.kProjectConstraint; //
  public final double MIN_PROJECTION = -MAX_PROJECTION - 1.0; // In inches with a offset because the pivot isn't in the
                                                              // center

  // Extender phyiscal numbers
  public final double L0 = 8.875; // inches - starting point, encoder zero -set 2/24/2019
  public final double STARTING_EXTENSION = L0; // inches - starting point, encoder zero -set 2/24/2019
  public final double EXTEND_MIN = 0.750; // inches 0.0 physic, .75 soft stop
  public final double EXTEND_MAX = 35.0; // inches - measured practice bot
  public final double ARM_BASE_LENGTH = 18.0; // inches - measured practice bot (from pivot center) xg 2/16/19
  public final double ARM_PIVOT_HEIGHT = 30.25; // inches - measured practice bot
  public final double WRIST_LENGTH = 4.5; // inches - measured practice bot 2/26/19
  public final double MAX_ARM_LENGTH = EXTEND_MAX + ARM_BASE_LENGTH + WRIST_LENGTH; // TODO: Find real max length of arm

  private final double kCounts_per_in = -607.0; // measured competition bot 4/24/2019
  private final double kIn_per_count = 1.0 / kCounts_per_in;

  // Coupling between Phi and extension, as arm moves, so does d
  private final double k_dl_dphi = 0.033415; // measured 2/24/2019 - practice bot new arm
  // 0.032639; // inches per degree (measured practice bot 2/17/19)

  // talon controls
  final int PIDIdx = 0; // using pid 0 on talon
  final int TO = 30; // timeout, we shouldn't really need one TODO try 0

  private long logTimer;

  public class Position {
    public double height; // inches above floor
    public double projection; // inches in front of pivot point
  };

  // outputs in robot coordinates h,ext (inches)
  Position position = new Position();

  private boolean extensionOverrided;

  /**
   * Creates a new arm/lift subsystem.
   */
  public ArmSubsystem() {
    addChild("Arm:Rot:Mtr", armRotationMotor);
    addChild("Arm:Ext:Mtr", armExtensionMotor);

    // Set Talon postion mode gains and power limits
    // Arm
    armRotationMotor.config_kP(0, 0.5);
    armExtensionMotor.config_kD(0, 4.0);
    armRotationMotor.configPeakOutputForward(0.3);
    armRotationMotor.configPeakOutputReverse(-0.3);
    armRotationMotor.configAllowableClosedloopError(0, 100);
    armRotationMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armRotationMotor.setInverted(true);

    // Extension on power will be out at L0.
    armExtensionMotor.config_kP(0, 0.6);
    armExtensionMotor.config_kD(0, 0.8);
    armExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armExtensionMotor.setIntegralAccumulator(0, 0, TO);
    armExtensionMotor.setSensorPhase(false);
    armExtensionMotor.setInverted(true);
    armExtensionMotor.configPeakOutputForward(0.5);
    armExtensionMotor.configPeakOutputReverse(-0.5);
    armExtensionMotor.configAllowableClosedloopError(0, 100);

    System.out.println("Warning - Arm Rotation has moderate Kp values & reduced power 30% limits");
    System.out.println("Warning - Arm Extension has moderate Kp values & reduced power 50% limits");
    logTimer = System.currentTimeMillis();

    zeroArm(); // will also get called on transition to teleOp, should arms be moved

    extensionOverrided = false;
    // set default command here since initdefault() is no longer a thing - dpl 12/17/2021
    setDefaultCommand(new ArmStatePositioner());
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /**
   * Zeros the arm's encoders - arm and extension are at starting point. Used by
   * commands at power up, or teleOp init.
   */
  public void zeroArm() {
    armExtensionMotor.setSelectedSensorPosition(0);
    armExtensionMotor.setIntegralAccumulator(0, PIDIdx, TO);

    armRotationMotor.setSelectedSensorPosition(0);
    armRotationMotor.setIntegralAccumulator(0.0, PIDIdx, TO);
  }

  /**
   * Resets the arm to match calculated position Run only when the arm extension
   * belt slips
   */
  public void resetArm(double resetLength) {
    double compLen = getCompLen(getRealAngle());
    double calculatedLength = resetLength - compLen - L0;

    int counts = (int) (calculatedLength * kCounts_per_in);
    armExtensionMotor.setSelectedSensorPosition(counts);
  }

  /**
   * Rotates the arm to a specific angle
   * 
   * @param angle the angle to rotate the arm to, in degrees
   */
  public void setAngle(double angle) {
    // If inverted, translate based on max angle backwards
    double counts = (PHI0 - angle) * kCounts_per_deg;
    armRotationMotor.set(ControlMode.Position, counts);
  }

  /**
   * Gets the angle at which the arm subsystem is rotated.
   * 
   * @return the angle of the arm, in degrees.
   */
  public double getRealAngle() {
    // return PHI_MAX - (rotationEncoder.getSelectedSensorPosition() / COUNT_MAX *
    // (PHI_MAX - PHI_MIN));
    double counts = armRotationMotor.getSelectedSensorPosition();
    double angle = PHI0 - counts * kDeg_per_count;
    return angle;
  }

  /**
   * Extends the extension to the length given. Compensate for the arm angle, phi,
   * so the desired extension is maintained.
   * 
   * Because the encoders are zeroed at PHI0 and L0 they must be accounted for...
   * At phi == phi0 there is no changes in length.
   * 
   * If we reset the talon with limit switch, we set new Phi0_l0 and L0
   * 
   * @param desired_L The desired extension of the Arm from beginning of the arm
   *                  housing to the back of the wrist motor
   */
  public void setExtension(double desired_L) {
    double angle = getRealAngle(); // current angle

    // Limit Extension
    double min_l_at_phi = getMinExtension(angle);
    double max_l_at_phi = getMaxExtension(angle);
    double limited_L = MathUtil.limit(desired_L, min_l_at_phi, max_l_at_phi);

    // Set extensionOverrided boolean
    extensionOverrided = !(Math.abs(limited_L - desired_L) <= 1e-6);

    // Print Warning
    if (desired_L < min_l_at_phi) {
      System.out.println("Desired Arm:Extension below minimum of " + min_l_at_phi + " inches.");
    } else if (desired_L > max_l_at_phi) {
      System.out.println("Desired Arm:Extension above maximum of " + max_l_at_phi + " inches.");
    }
    

    SmartDashboard.putNumber("Extension Calculated", limited_L);

    // Adjust length to match L0 and account for compLen
    double compLen = ((angle - PHI0) * k_dl_dphi); // ext due to rotation to compensate for
    double cmd_L = limited_L - L0 - compLen;
    SmartDashboard.putNumber("Extension Compensation", compLen);
    SmartDashboard.putNumber("Extension Set", cmd_L);

    double c = cmd_L * kCounts_per_in;
    armExtensionMotor.set(ControlMode.Position, c);
  }

  /**
   * Returns the minimal extention in inches based on an arm angle in degrees.
   * Vaild over full range of phi
   * 
   * @param angle
   * @return
   */
  public double getMinExtension(double angle) {
    if(-32.0 < angle && angle < 32.0) {
      return STARTING_EXTENSION;
    }
    return EXTEND_MIN;
  }

  /**
   * Returns the max extension based on arm angle. PhiCrit is the cutoff where we
   * need to make sure we limit our max length. Less than PhiCrit we can have full
   * extension and stay in the box.
   * 
   * Valid over full range of Phi, except for minor error if we are on the
   * backside of the robot. On backside, the MAX_PROJECTION is off by about 1/2
   * inch.
   * 
   * @param angle
   * @return
   */
  public double getMaxExtension(double angle) {
    final double PhiCrit = Math.toDegrees(Math.asin((MAX_PROJECTION) / (WRIST_LENGTH + ARM_BASE_LENGTH + EXTEND_MAX)));

    double absAngle = Math.abs(angle);
    if (absAngle < PhiCrit) {
      return EXTEND_MAX;
    }
    return (MAX_PROJECTION / Math.sin(Math.toRadians(absAngle))) - ARM_BASE_LENGTH - WRIST_LENGTH;
  }

  /**
   * Gets the extension length of the extension (l) in inches. This is not the
   * total arm lenght, it is just the extension.
   * 
   * @return extension (desired_l), in inches.
   */
  public double getExtension() {
    double counts = armExtensionMotor.getSelectedSensorPosition();   // TODO: confirm we still get counts even if a double not int DPL 12/17/2021
    // L0 + phi correction
    return (L0 + (getRealAngle() - PHI0) * k_dl_dphi) + (counts * kIn_per_count);
  }

  /**
   * Gets whether or not the arm is at the state of being the least extended it
   * can. Active low on the wiring, we invert the signal
   * 
   * @return <code>true</code> if the arm is at the minimum extension state,
   *         <code>false</code> otherwise.
   */
  public boolean extensionAtMin() {
    // this signal is active low
    return !extensionAtMin.get();
  }

  /**
   * Gets whether or not the arm is at the state of being the most extended it
   * can.
   * 
   * @return <code>true</code> if the arm is at the maximum extension state,
   *         <code>false</code> otherwise.
   */
  public boolean extensionAtMax() {
    return armExtensionMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public double getHeight() {
    double height = Math.cos(Math.toRadians(getRealAngle())) * (getExtension() + ARM_BASE_LENGTH + WRIST_LENGTH) + ARM_PIVOT_HEIGHT;
    return height;
  }

  public double getProjection() {
    double projection = Math.sin(Math.toRadians(getRealAngle())) * (getExtension() + ARM_BASE_LENGTH + WRIST_LENGTH);
    return projection;
  }

  /**
   * Computes height of gripper and projection on floor from pivot, pivot is
   * horizontal zero
   */
  public Position getArmPosition() {
    double phi = getRealAngle();
    double rads = Math.toRadians(phi);
    double ext = getExtension(); // includes angle compensation
    double desired_l = ARM_BASE_LENGTH + WRIST_LENGTH + ext;
    position.height = ARM_PIVOT_HEIGHT + desired_l * Math.cos(rads);
    position.projection = desired_l * Math.sin(rads);
    return position;
  }

  // Expected change in extension as a result of phi, phi in degrees
  public double getCompLen(double phi) {
    // base + wrist + phi-rotation
    double compLen = ((phi - PHI0) * k_dl_dphi);
    return (compLen);
  }

  public ArmStatePositioner getArmPositioner() {
    return (ArmStatePositioner) getDefaultCommand();
  }

  /**
   * Gets whether the arm is inverted
   * 
   * @return Inversion status
   */
  public boolean isInverted() {
    return getRealAngle() < 0;
  }

  public boolean isExtensionOverrided() {
    return extensionOverrided;
  }
  

  /**
   * Gets the command used to zero the arm subsystem.
   * 
   * @return a new <code>ArmZero</code> command.
   */
  public Command zeroSubsystem() {
    return new ArmZero();
  }

  public void log(int interval) {
    if ((logTimer + interval) < System.currentTimeMillis()) { // only post to smartdashboard every interval ms
      logTimer = System.currentTimeMillis();

      // SmartDashboard.putData((Sendable) armRotationMotor);
      // SmartDashboard.putData((Sendable) armExtensionMotor);
      SmartDashboard.putNumber("Arm:Phi(raw)", armRotationMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm:Phi(deg)", getRealAngle());
      SmartDashboard.putNumber("Arm:Ext(raw)", armExtensionMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm:Ext(in)", getExtension());
      SmartDashboard.putNumber("Arm:Gripper Projection", getArmPosition().projection);
      SmartDashboard.putNumber("Arm:Gripper Height", getArmPosition().height);

      // don't have limit switches right now
      SmartDashboard.putBoolean("Arm:Ext@Min", extensionAtMin());
      // SmartDashboard.putBoolean("Arm:Ext@Max", extensionAtMax());
    }
    return;
  }

  /**
   * Logs the talons used in the arm subsystem, which are the rotation motor and
   * the extension motor.
   */
  public void logTalons() {
    logTalon(armRotationMotor);
    logTalon(armExtensionMotor);
  }

  /**
   * Logs the given talon.
   * 
   * @param talon the WPI_TalonSRX to log.
   */
  private void logTalon(WPI_TalonSRX talon) {
    SmartDashboard.putNumber(talon.getBaseID() + " Current", talon.getSupplyCurrent() );
    SmartDashboard.putNumber(talon.getBaseID() + " Percent Output", talon.getMotorOutputPercent());
  }

}
