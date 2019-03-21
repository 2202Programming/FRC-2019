package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CheckSolenoids extends Command {
    public CheckSolenoids() {
        requires(Robot.climber);
    }
    
    protected void initialize() {
        setTimeout(0.1);
        Robot.climber.setDrawerSlide(true);
        Robot.climber.setPawl(true);
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end() {
        Robot.climber.setDrawerSlide(false);
        Robot.climber.setPawl(false);
    }
}