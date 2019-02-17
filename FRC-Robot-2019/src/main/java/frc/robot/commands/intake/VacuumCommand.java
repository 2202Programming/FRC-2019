package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 *   VacuumCommand (enable)  
 *       true - turn it on
 *       false - turn it off
 * 
 *      Instantiate with one or other for what you need.
 */
public class VacuumCommand extends InstantCommand {
    boolean enable;
    public VacuumCommand(boolean enable) {
        requires(Robot.intake.getVacuumSubsystem());
        this.setName("vac="+enable);
        this.enable = enable;
    }

    @Override
    protected void execute() {
        if (enable) Robot.intake.vacuumOn();
        else        Robot.intake.vacuumOff();    
    }
}
