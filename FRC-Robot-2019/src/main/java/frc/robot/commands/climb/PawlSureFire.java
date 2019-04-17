package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;
/**
 * PawlSureFire - puts the Pawl in the requested direction but
 * first fires the motor in the opp direction to offload the gear.
 * 
 */
public class PawlSureFire extends Command {
    DoubleSolenoid.Value cmd;
    int kCount;
    int frameCount;
    boolean done = false;

    public PawlSureFire(DoubleSolenoid.Value cmd, int kCount) {
        this.cmd = cmd;
        this.kCount = kCount;
        requires(Robot.climber);
    }
    
    protected void initialize() {
        frameCount = kCount;
        done = false;
    }

    protected void execute() {
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

    protected boolean isFinished() {
        return done;
    }
}