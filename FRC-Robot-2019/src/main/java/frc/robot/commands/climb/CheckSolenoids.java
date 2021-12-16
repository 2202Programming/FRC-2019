package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CheckSolenoids extends CommandBase {
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