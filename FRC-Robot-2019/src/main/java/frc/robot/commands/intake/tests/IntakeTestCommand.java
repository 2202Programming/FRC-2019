package frc.robot.commands.intake.tests;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class IntakeTestCommand extends InstantCommand {
    // Current state
    boolean vacuumOn;

    public IntakeTestCommand(boolean vacuumOn) {
        requires(Robot.intake.getVacuumSubsystem());
        this.setName("vac=" + vacuumOn);
        this.vacuumOn = vacuumOn;
    }

    @Override
    protected void execute() {
        Robot.intake.setVacuum(vacuumOn);
    }
}
