package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * Stops the suctions cup's solenoid from being stuck in indeterminant state
 */
public class CheckSucc extends CommandBase {
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