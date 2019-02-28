package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class ClimbSolenoidTestCmd extends InstantCommand {
    // Current state
    boolean enabled;

    public ClimbSolenoidTestCmd(boolean enabled) {
        requires(Robot.climber);
        this.setName("vac=" + enabled);
        this.enabled = enabled;
    }

    @Override
    protected void execute() {
        if (enabled) {
            // Transition to Off
            Robot.climber.setRatchetExtend(false);;
        } else {
            // Transition to On
            Robot.climber.setRatchetExtend(true);
        }
        enabled = !enabled;
    }
}
