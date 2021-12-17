package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;
/**
 * PawlSureFire - puts the Pawl in the requested direction but
 * first fires the motor in the opp direction to offload the gear.
 * 
 */
public class PawlSureFire extends CommandBase {
    DoubleSolenoid.Value cmd;
    int kCount;
    int frameCount;
    boolean done = false;

    public PawlSureFire(DoubleSolenoid.Value cmd, int kCount) {
        this.cmd = cmd;
        this.kCount = kCount;
        addRequirements(Robot.climber);
    }
    
   public void initialize() {
        frameCount = kCount;
        done = false;
    }

    public void execute() {
        // run motor opp of what we want to unload before switching

        if (cmd == Robot.climber.Extend) {
            Robot.climber.setExtenderSpeed(Robot.climber.STALL_POWER_RETRACT);
        }
        else {  //Robot.climber.Retract
            Robot.climber.setExtenderSpeed(Robot.climber.STALL_POWER_EXTEND);
        }
        
        if (frameCount <=1) {
            Robot.climber.setPawl(cmd);
        } 
        
        if(frameCount <= 0) {
            Robot.climber.setExtenderSpeed(0.0);
            done = true;
        }
        frameCount--;
    }

    public boolean isFinished() {
        return done;
    }
}