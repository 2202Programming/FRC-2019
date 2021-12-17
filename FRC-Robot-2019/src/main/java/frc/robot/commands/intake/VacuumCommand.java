package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 *   VacuumCommand (enable)  
 *       true - turn it on - effectively an instanct
 *       false - turn it off  - will run duration of timeout
 *             - leaves the pump off, but will put solenoid to vac mode
 *               so compressed air won't drain.
 * 
 *      Instantiate with one or other for what you need.
 */
public class VacuumCommand extends CommandBase {
    boolean enable;
    boolean done = false;
    double timeout;

    public VacuumCommand(boolean enable, double timeout) {
        addRequirements(Robot.intake.getVacuumSubsystem());
        this.setName("vac="+enable);
        this.enable = enable;
        this.timeout = timeout;
    }

    @Override
   public void initialize()
    {
        done = false;          // turning off takes time.
        setTimeout(timeout); 

        if (enable) {
            done = true;              // this is instant
            Robot.intake.setVacuum(true);
        }
        else {
            //pump off, flip the solenoid to put compressed air into lines
            Robot.intake.setVacuum(false);
            Robot.intake.releaseSolenoid(true);  //powered, will release payload
        }
    }

    @Override
    public void execute() {
        //just wait for time out or we are done.
    }

    // When we release we blow air when we retract, sensor goes below baseline - problem...
    @Override
    public boolean isFinished() {
        // put the solenoid back to vacuum on timeout.
        if (isTimedOut()) {
            Robot.intake.releaseSolenoid(false);  //unpowerered for vacuum
        }
        return done || isTimedOut();
    }

}
