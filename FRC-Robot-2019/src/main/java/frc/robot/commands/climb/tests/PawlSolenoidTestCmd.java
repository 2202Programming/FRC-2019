package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class PawlSolenoidTestCmd extends InstantCommand {
    // On state
    boolean enabled;

    public PawlSolenoidTestCmd(boolean enabled) {
        this.setName("vac=" + enabled);
        this.enabled = enabled;
    }

    @Override
    protected void initialize() {
        Robot.climber.setPawl(enabled);
    }

    @Override
    protected void execute() {
        Robot.climber.setPawl(enabled);
    }

    @Override
    protected void end() {
        Robot.climber.setPawl(!enabled);        
    }
}
