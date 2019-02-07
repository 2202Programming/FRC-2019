package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The basic intake subsystem
 */
public class IntakeSubsystem extends Subsystem {
  /**
   * A servo that rotates the "wrist" of the arm of the robot.
   */
  private Servo wristRotation = new Servo(RobotMap.WRIST_SERVO_PWM_CH);
  private double wristPosition = wristRotation.get();

  /**
   * A switch that detects whether or not the robot is holding any cargo.
   */
  private DigitalInput cargoSwitch = new DigitalInput(RobotMap.CARGO_SENSOR_DIO_PORT);

  /**
   * A vacuum pump that holds onto the cargo.
   */
  private SpeedController vacuumPump = new Spark(RobotMap.VACUUM_SPARK_PIN);

  private Solenoid release = new Solenoid(RobotMap.RELEASE_SOLENOID_ID);

  /**
   * Creates an intake subsystem.
   */
  public IntakeSubsystem() {
    addChild("Wrist Rotation Servo", (Sendable) wristRotation);
    addChild("Vacuum Pump", (Sendable) vacuumPump);
    addChild("Cargo/Hatch Release", (Sendable) release);
  }

  @Override
  public void initDefaultCommand() {
    //funstuffyebrocool!
    //(No default command, apparently)
  }

  /**
   * Sets the position of the wrist (servo) to the given value.
   * Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
   * @param position the position value of the wrist.
   */
  public void setWristPosition(double position) {
    wristPosition = position;
    wristRotation.set(position);
  }

  public void runWristUp() {
    wristPosition += 0.01;
    wristRotation.set(wristPosition);
  }

  public void runWristDown() {
    wristPosition -= 0.01;
    wristRotation.set(wristPosition);
  }

  public void stopWrist() {
    wristRotation.set(wristPosition);
  }

  /**
   * Sets the vacuum pump to the given speed. Speed values range from -1.0 to 1.0.
   * @param speed the speed value to set the vacuum pump.
   */
  public void runPump(double speed) {
    vacuumPump.set(speed);
  }

  /**
   * Runs the intake system. This will turn on the release solenoid and set the vacuum pump's speed to 0.5.
   */
  public void run() {
    double pumpSpeed = .5;
    release.set(true);
    runPump(pumpSpeed);
  }

  /**
   * Stops the intake subsystem, causing the vacuum pump to have a speed of 0 and turning the release solenoid off.
   */
  public void stop() {
    vacuumPump.set(0);
    release.set(false);
  }

  /**
   * Gets the status of the cargo switch (whether or not the robot is holding cargo).
   * @return true if the robot is holding cargo; false otherwise.
   */
  public boolean getCargoSwitch() {
    return cargoSwitch.get();
  }
}