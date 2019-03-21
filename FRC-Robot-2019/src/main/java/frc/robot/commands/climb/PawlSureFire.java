package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PawlSureFire extends Command {
    boolean on;
    public PawlSureFire(boolean on) {
        this.on = on;
        requires(Robot.climber);
    }
    
    protected void initialize() {
        setTimeout(0.1);
    }

    protected void execute() {
        if (on) Robot.climber.setExtenderSpeed(Robot.climber.STALL_POWER_EXTEND);
        else Robot.climber.setExtenderSpeed(Robot.climber.STALL_POWER_RETRACT);
        Robot.climber.setPawl(on);
    }

    protected boolean isFinished() {
        return isTimedOut();
    }
}