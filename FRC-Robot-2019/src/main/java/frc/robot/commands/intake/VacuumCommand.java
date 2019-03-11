package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *   VacuumCommand (enable)  
 *       true - turn it on - effectively an instanct
 *       false - turn it off  - will run duration of timeout
 * 
 *      Instantiate with one or other for what you need.
 */
public class VacuumCommand extends Command {
    boolean enable;
    boolean done = false;
    double timeout;

    public VacuumCommand(boolean enable, double timeout) {
        requires(Robot.intake.getVacuumSubsystem());
        this.setName("vac="+enable);
        this.enable = enable;
        this.timeout = timeout;
    }

    @Override
    protected void initialize()
    {
        done = false;      // turning off takes time.
        setTimeout(timeout);   //todo: not sure how much
    }

    @Override
    protected void execute() {
        if (enable) {
            done = true;              // this is instant
            Robot.intake.vacuumOn();
        }
        else {
            //keep the pump on, flip the solenoid
            Robot.intake.releaseSolenoid(Robot.intake.kRelease);  //powered
        }

    }

    @Override
    protected boolean isFinished() {
        // put the solenoid back to vacuum on timeout.
        if (isTimedOut()) {
            Robot.intake.releaseSolenoid(Robot.intake.kVacuum);  //unpowerered
        }
        return done || isTimedOut();
    }
    

}
