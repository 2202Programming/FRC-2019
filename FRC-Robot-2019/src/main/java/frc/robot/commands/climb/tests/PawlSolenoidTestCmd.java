package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * 
 * Instantiate with one or other for what you need.
 */
public class PawlSolenoidTestCmd extends Command {
    // On state
    boolean enabled;

    public PawlSolenoidTestCmd(boolean enabled) {
        this.setName("PawlTest=" + enabled);
        this.enabled = enabled;
    }

    @Override
    protected void initialize() {
        Robot.climber.setPawl(enabled);
    }

    @Override
    protected void execute() {      
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
