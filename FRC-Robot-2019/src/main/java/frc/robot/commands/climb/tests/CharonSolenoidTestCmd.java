package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class CharonSolenoidTestCmd extends InstantCommand {
    // On state
    boolean enabled;

    public CharonSolenoidTestCmd(boolean enabled) {
        this.setName("vac=" + enabled);
        this.enabled = enabled;
    }

    @Override
    protected void initialize() {
        Robot.climber.setDrawerSlide(enabled);
    }

    @Override
    protected void execute() {
        Robot.climber.setDrawerSlide(enabled);
    }

    @Override
    protected void end() {
        Robot.climber.setDrawerSlide(!enabled);        
    }
}
