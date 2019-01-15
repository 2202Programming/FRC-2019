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
    private SpeedController intakeLeftMotor = new Spark(RobotMap.LEFT_INTAKE_MOTOR_PIN);
    private SpeedController intakeRightMotor = new Spark(RobotMap.RIGHT_INTAKE_MOTOR_PIN);
    private DigitalInput photoGate = new DigitalInput(RobotMap.INTAKE_PHOTOGATE_CHANNEL);
    public IntakeSubsystem() {
  
      addChild("Intake Left Spark", (Sendable) intakeLeftMotor);
      addChild("Intake Right Spark", (Sendable) intakeRightMotor);

    }
  
    @Override
    public void initDefaultCommand() {
       //funstuffyebrocool!
    }
    
    public void outtake(){

    }
}