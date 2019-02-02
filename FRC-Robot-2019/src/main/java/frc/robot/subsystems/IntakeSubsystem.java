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

    private Servo wristRotation = new Servo(RobotMap.WRIST_SERVO_PWM_CH);
    private DigitalInput cargoSwitch = new DigitalInput(RobotMap.CARGO_SENSOR_DIO_PORT);
    private SpeedController vacuumPump = new Spark(RobotMap.VACUUM_SPARK_PIN);
    private Solenoid release = new Solenoid(RobotMap.RELEASE_SOLENOID_ID);

    public IntakeSubsystem() {
      addChild("Wrist Rotation Servo", (Sendable) wristRotation);
      addChild("Vacuum Pump", (Sendable) vacuumPump);
      addChild("Cargo/Hatch Release", (Sendable) release);
    }
  
    @Override
    public void initDefaultCommand() {
       //funstuffyebrocool!
    }

    public void setWristPosition(double position)
    {
      wristRotation.set(position);
    }

    public void runPump(double speed) {
      vacuumPump.set(speed);
    }

    public void run()
    {
      double placeHolderValue = .5;
      release.set(true);
      runPump(placeHolderValue);
    }
  
    public void stop()
    {
      vacuumPump.set(0);
      release.set(false);
    }

    public boolean getCargoSwitch(){
      return cargoSwitch.get();
    }
}