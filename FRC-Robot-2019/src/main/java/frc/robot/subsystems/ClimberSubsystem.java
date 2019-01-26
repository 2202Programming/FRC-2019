package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Spark;
import frc.robot.RobotMap;

public class ClimberSubsystem extends Subsystem {
    private SpeedController climbMotor = new Spark(RobotMap.CLIMBER_MOTOR_PIN);
    private Solenoid sole = new Solenoid(RobotMap.CLIMBER_SOLENOID_CAN_ID);
    
	public ClimberSubsystem() {
        addChild("Climber Spark", (Sendable) climbMotor);
        addChild("Climber CAN", (Sendable) sole);
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
    public void activate() {
        sole.set(true);
    }
    public void deactivate() {
        sole.set(false);
    }
    @Override
	public void initDefaultCommand(){
        //nice stuff bruv
	}
}