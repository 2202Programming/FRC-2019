package frc.robot.commands.intake.tests;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class SolenoidTestCommand extends InstantCommand {
    // Current state
    boolean enabled;

    public SolenoidTestCommand(boolean enabled) {
        requires(Robot.intake.getVacuumSubsystem());
        this.setName("vac=" + enabled);
        this.enabled = enabled;
    }

    @Override
    protected void execute() {
        if (enabled) {
            // Transition to Off
            Robot.intake.releaseSolenoid(true);
        } else {
            // Transition to On
            Robot.intake.releaseSolenoid(false);
        }
        enabled = !enabled;
    }
}
