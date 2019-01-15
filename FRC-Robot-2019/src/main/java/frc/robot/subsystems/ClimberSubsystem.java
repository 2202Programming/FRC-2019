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

public class ClimberSubsystem extends Subsystem {
    private SpeedController climbMotor = new Spark(RobotMap.CLIMBER_MOTOR_PIN);
	
	public ClimberSubsystem() {
        addChild("Climber Spark", (Sendable) climbMotor);
    }

    @Override
	public void initDefaultCommand(){
        //nice stuff bruv
	}
}