package frc.robot.subsystems;

import frc.robot.commands.TankDriveCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.RobotMap;

/**
 * The basic intake subsystem
 */
public class IntakeSubsystem extends Subsystem {

    // Individual Motors
    private SpeedController intakeMotorLeft = new Spark(RobotMap.LEFT_INTAKE_MOTOR_PIN);
    private SpeedController intakeMotorRight = new Spark(RobotMap.RIGHT_INTAKE_MOTOR_PIN);
    private DigitalInput photoGate = new DigitalInput(RobotMap.INTAKE_PHOTOGATE_CHANNEL);
    public IntakeSubsystem() {
      addChild("Intake Left Spark", (Sendable) intakeMotorLeft);
      addChild("Intake Right Spark", (Sendable) intakeMotorRight);
    }
  
    @Override
    public void initDefaultCommand() {
       //funstuffyebrocool!
    }

    public void runIntake(double speed) {
      intakeMotorLeft.set(speed);
      intakeMotorRight.set(-speed);
    }
  
    public void intake() {
      intakeMotorLeft.set(0.8);
      intakeMotorRight.set(-0.6);
    }
  
    public void outtake() {
      intakeMotorLeft.set(-0.6);
      intakeMotorRight.set(0.6);
    }
  
    public void outtakeSlow() {
      intakeMotorLeft.set(-0.3);
      intakeMotorRight.set(0.3);
    }
  
    public void rotate() {
      intakeMotorLeft.set(0.3);
      intakeMotorRight.set(0.3);
    }
  
    public void holdBlock() {
      intakeMotorLeft.set(0.2);
      intakeMotorRight.set(-0.2);
    }
  
    public void stop() {
      intakeMotorLeft.set(0);
      intakeMotorRight.set(0);
    }

    public boolean getPhotoGate(){
      return photoGate.get();
    }
}