package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Stops the suctions cup's solenoid from being stuck in indeterminant state
 */
public class CheckSucc extends Command {
    public CheckSucc() {
        requires(Robot.intake);
    }
    
    protected void initialize() {
        setTimeout(0.2);
        Robot.intake.releaseSolenoid(true);
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end() {
        Robot.intake.releaseSolenoid(false);
    }
}