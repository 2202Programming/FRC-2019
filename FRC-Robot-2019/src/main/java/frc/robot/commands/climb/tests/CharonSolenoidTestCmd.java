package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * 
 * Instantiate with one or other for what you need.
 */
public class CharonSolenoidTestCmd extends Command {
    // On state
    boolean enabled;

    public CharonSolenoidTestCmd(boolean enabled) {
        this.setName("Charon=" + enabled);
        this.enabled = enabled;
    }

    @Override
    protected void initialize() {
       Robot.climber.setDrawerSlide(enabled); 
    }
    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
