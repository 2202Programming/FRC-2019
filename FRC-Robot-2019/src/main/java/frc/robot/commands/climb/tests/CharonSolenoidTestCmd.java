package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
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
    protected void execute() {
        Robot.climber.setDrawerSlide(enabled);
    }

    @Override
    protected void end() {
        System.out.println("Entered End of CharonCommand");
        Robot.climber.setDrawerSlide(!enabled);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
