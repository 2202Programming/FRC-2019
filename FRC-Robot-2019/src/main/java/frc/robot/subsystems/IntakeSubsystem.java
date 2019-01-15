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
 * The basic intake subsystem
 */
public class IntakeSubsystem extends Subsystem {

    // Individual Motors
    private SpeedController IntakeLeftMotor = new Spark(RobotMap.LEFT_INTAKE_MOTOR_PIN);
    private SpeedController IntakeRightMotor = new Spark(RobotMap.RIGHT_INTAKE_MOTOR_PIN);
  
    public IntakeSubsystem() {
  
      addChild("Intake Left Spark", (Sendable) IntakeLeftMotor);
      addChild("Intake Right Spark", (Sendable) IntakeRightMotor);

    }
  
    @Override
    public void initDefaultCommand() {
       //funstuffyebrocool!
    }
    
}