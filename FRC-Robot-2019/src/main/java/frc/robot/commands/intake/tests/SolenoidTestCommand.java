package frc.robot.commands.intake.tests;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 *  Use Robot.intake.kRelease or kVacuum 
 * 
 */
public class SolenoidTestCommand extends InstantCommand {
    // Current state
    boolean vacuumCmd;

    public SolenoidTestCommand(boolean vacuumCmd) {
        requires(Robot.intake.getVacuumSubsystem());
        this.setName("solenoidTest=" + vacuumCmd);
        this.vacuumCmd = vacuumCmd;
    }

    @Override
    protected void execute() {
        Robot.intake.releaseSolenoid(vacuumCmd);
    }
}
