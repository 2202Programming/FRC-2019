package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.subsystems.ExtendedSubSystem;
import frc.robot.RobotMap;

//used for CustomServo
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Authors: Derek Laufenberg
 *          Billy Huang
 *          Alexander Ge
 *         
 * Changes:  
 * 2/10/2019   DPL  Added notes, cleaned up code, created/fixed low level controls 
 *                  to use for inside commands.
 *                  Added constantant, improved nameing conventions.
 *                  Use DoubleSolenoid.
 *                  
 * 2/17/2019   DPL  Split vacuum into own subsystem, but contained here
 * 
 */
/**
 * The basic intake subsystem for Cargo and Hatch pickup.
 * 
 * Consists of of 550 motor driving a vacuum pump to draw air on a suction cup.
 * The suction cup has a switch in it so it can dectect a cargo ball. The cargo
 * ball deforms slightly to trigger the swtich.
 * 
 * A composite solenoid is used to turn on and off the vacuum line to the
 * suction cup. These need to be controled with the pump motor.
 * 
 * A servo based "Wrist" is used to rotate the suction cup to make it easier to
 * align with the hatch or cargo location.
 * 
 */

public class IntakeSubsystem extends ExtendedSubSystem {
  // Local Constants that define facts about the intake system
  public final double WristMinDegrees = -100.0; // pointing down, relative to the arm //dpl hack
  public final double WristMaxDegrees = +100.0; // pointing up
  public final double WristStraightDegrees = 0.0; // points near straight out
  public final double WristDistToPivot = 4.0; //inches
  public final double PumpSpeed = 1.0; // motor units
  private long logTimer;

  // ### Servo range needs to be checked since the servo is modified with an
  // external
  // ### pot that is part of the ServoCity gearing kit.

  // HS-805MG - timings from https://www.servocity.com/hs-805mg-servo
  // specs are slightly wider than default which should give us full range of
  // servo.
  final double kServoMinPWM = 0.553;
  final double kServoMaxPWM = 2.455;

  Subsystem intakeVacuum;

  // TODO: confirm this is the way the solenoid is wired
  final DoubleSolenoid.Value kVacuum = Value.kForward;
  final DoubleSolenoid.Value kRelease = Value.kReverse;

  // Physical devices
  CustomServo wristServo;             // positive angle, wrist up, when arm is forward
  DigitalInput cargoSwitch;           // true when cargo switch is pressed by ball
  SpeedController vacuumPump;         // motor control for vacuum pump
  DoubleSolenoid vacuumSol;           // solenoid to hold and relase ball/hatch

  /**
   * Creates an intake subsystem.
   */
  public IntakeSubsystem() {
    super("Intake");
    wristServo = new CustomServo(RobotMap.INTAKE_WRIST_SERVO_PWM, WristMinDegrees, WristMaxDegrees,
    kServoMinPWM, kServoMaxPWM);
    wristServo.setName(this.getSubsystem(), "wrist");

    cargoSwitch = new DigitalInput(RobotMap.INTAKE_CARGO_SWITCH_MXP_CH);
    vacuumPump = new Spark(RobotMap.INTAKE_VACUUM_SPARK_PWM);
    vacuumSol = new DoubleSolenoid(RobotMap.INTAKE_PCM_ID, RobotMap.INTAKE_RELEASE_SOLENOID_PCM,
    RobotMap.INTAKE_HOLD_SOLENOID_PCM);

    // addChild("In:Wrist", (Sendable) wristServo);
    // addChild("In:VacPump", vacuumPump);
    addChild("In:CargoSw", cargoSwitch);
    addChild("In:VacSol",  vacuumSol);

    intakeVacuum = new Subsystem("Intake:Vac"){
      @Override
      protected void initDefaultCommand() {}  //none 
    };

    logTimer = System.currentTimeMillis();
  }

  @Override
  public void initDefaultCommand() {
  }

  /**
   * Wrist Controls - With the Arm in front,
   *                 positive -->angle up
   *                 negitive -->angle down
   *                 0.0 --> level with arm mount
   */
  public void setAngle(double degrees) { wristServo.setAngle(-degrees);  }

  /**
   * -1*servo angle is correct by our convention.
   * @return wrist angle (degrees)
   */
  public double getAngle() {  return -wristServo.getAngle();  }

  /**
   *  Commands can used the vacuum subsystem without interferring with wrist.
  */
  public Subsystem getVacuumSubsystem()  {
    return this.intakeVacuum;
  }

  /**
   * Vacuum Controls
   */
  public void vacuumOn() {
    vacuumPump.set(PumpSpeed);
    vacuumSol.set(kVacuum);
  }

  public void vacuumOff() {
    vacuumSol.set(kRelease);
    vacuumPump.stopMotor();
  }

  public void vacuumOffHoldOn() {
    vacuumSol.set(kVacuum);   // keep the vacuum.  
    vacuumPump.stopMotor();
  }

  public boolean isVacuum() {
    boolean v =  (vacuumSol.get() == kVacuum);
    return v;
  }

  /**
   * Status and Sensor Methods
   */

  private void zeroIntake() {
    vacuumOff();
  }

  /**
   * Gets the status of the cargo switch (whether or not the robot is holding
   * cargo).
   * 
   * @return true if the robot is holding cargo; false otherwise.
   */
  public boolean hasCargo() {
    return cargoSwitch.get();
  }

  // ### check the PDP for current level, we can tell if vacuum is obtained.
  public double getPumpCurrent() {
    return -999.999;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    //builder.setSmartDashboardType("CustomServo");
    //builder.addDoubleProperty("Value", this::getAngle, this::setAngle);
  }

  // Create the zeroSubsystem Command
  @Override
  public Command zeroSubsystem() {
    // local class for zero
    class IntakeZeroCmd extends Command {
      IntakeZeroCmd(Subsystem ss) {
        requires(ss);
      }

      @Override
      protected void initialize() {
        zeroIntake();
      }
      @Override
      protected boolean isFinished() { return true; }
    }
    // create the zero command we just defined
    return new IntakeZeroCmd(this);
  }

  public void log() {
    SmartDashboard.putData("intake0", this);
    SmartDashboard.putNumber("In:Wr(deg)", getAngle());
  }

  /**
   * The Servo used by the Wrist has been modified to extend the range. It also
   * has a wider PWM setting than the standard FIRST Servo Class. So that code was
   * modified to create a CustomServo that we can change the PWM controls on.
   * 
   * We also changed the Servo 0-180 to -100 to +100 which reflects the server
   * travel and the Wrist axis with ~0.0 degrees being straight from arm.
   * 
   * The Servo class is further modified to only expose the methods that use the
   * degree units. This is to save confusion as to what units should be used.
   * 
   * PWM class tracks state on 0 to 1.0 range.
   * 
   * Update Notes 2/10/2019 DPL Created from wpilibj.Servo base code. Couldn't
   * inherit from servo.
   * 
   */
  private class CustomServo extends PWM {
    private final double kMaxServoAngle;
    private final double kMinServoAngle;
    private final double kServoRange;

    private final double kDefaultMaxServoPWM;
    private final double kDefaultMinServoPWM;
    private double position;   // save the setting and use it to return a value if asked
       
    /**
     * Constructor.<br>
     *
     * <p>
     * By default {@value #kDefaultMaxServoPWM} ms is used as the maxPWM value<br>
     * By default {@value #kDefaultMinServoPWM} ms is used as the minPWM value<br>
     *
     * @param channel    The PWM channel to which the servo is attached. 0-9 are
     *                   on-board, 10-19 are on the MXP port
     * 
     * @param minDegrees smallest angle of rotation in degrees
     * 
     * @param maxDegrees largest angle of rotation in degrees
     * 
     * @param minPWMuS   servo timing min value .6 typical
     * @param maxPWMuS   Servo timing max, 2.5 uS typical
     * 
     */
    public CustomServo(final int channel, final double minDegrees, final double maxDegrees, final double minPWMuS,
        final double maxPWMuS) {
      super(channel);
      kMaxServoAngle = maxDegrees;
      kMinServoAngle = minDegrees;
      kDefaultMaxServoPWM = maxPWMuS;
      kDefaultMinServoPWM = minPWMuS;
      // compute range once
      kServoRange = kMaxServoAngle - kMinServoAngle;  
      setBounds(kDefaultMaxServoPWM, 0.0, 0.0, 0.0, kDefaultMinServoPWM);
      setPeriodMultiplier(PeriodMultiplier.k4X);

      HAL.report(tResourceType.kResourceType_Servo, getChannel());
      setName("CustomServo", getChannel());
    }

    /**
     * Set the servo angle.
     *
     * <p>
     * Assume that the servo angle is linear with respect to the PWM value (big
     * assumption, need to test).
     *
     * <p>
     * Servo angles that are out of the supported range of the servo simply
     * "saturate" in that direction In other words, if the servo has a range of (X
     * degrees to Y degrees) than angles of less than X result in an angle of X
     * being set and angles of more than Y degrees result in an angle of Y being
     * set.
     *
     * @param degrees The angle in degrees to set the servo.
     */
    public void setAngle(double degrees) {
      if (degrees < kMinServoAngle) {
        degrees = kMinServoAngle;
      } else if (degrees > kMaxServoAngle) {
        degrees = kMaxServoAngle;
      }

      position = (degrees - kMinServoAngle) / kServoRange;
      setPosition(position);
    }

    /**
     * Get the servo angle.
     *
     * <p>
     * Assume that the servo angle is linear with respect to the PWM value (big
     * assumption, need to test).
     * 
     * Derek L - getPosition() returns zero.  Fake a value with saved position. 2/24/2019
     *
     * @return The angle in degrees to which the servo is set.
     */
    public double getAngle() {
      double pos =  getPosition() * kServoRange + kMinServoAngle;
      return pos;
    }

    // DPL - use only the get/set angle for CustomServo
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("CustomServo");
      builder.addDoubleProperty("Value", this::getAngle, this::setAngle);
    }
  }

  public void log(int interval) {

    if ((logTimer + interval) < System.currentTimeMillis()) { //only post to smartdashboard every interval ms
      logTimer = System.currentTimeMillis();

      SmartDashboard.putNumber("In:Wr(deg)", getAngle());

    }
  }

}