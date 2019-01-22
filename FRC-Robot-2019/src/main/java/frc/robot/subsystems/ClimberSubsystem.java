package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Spark;
import frc.robot.RobotMap;

public class ClimberSubsystem extends Subsystem {
    private SpeedController climbMotor = new Spark(RobotMap.CLIMBER_MOTOR_PIN);
    
	public ClimberSubsystem() {
        addChild("Climber Spark", (Sendable) climbMotor);
    }

    public void climbSlow () {
        climbMotor.set(0.5);
    }

    public void climbFast () {
        climbMotor.set(1);
    }

    public void climbReverse () {
        climbMotor.set(-0.5);
    }
    public void stopClimb() {
        climbMotor.set(0);
    }
    @Override
	public void initDefaultCommand(){
        //nice stuff bruv
	}
}