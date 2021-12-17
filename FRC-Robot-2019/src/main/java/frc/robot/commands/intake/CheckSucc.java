package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

/**
 * Stops the suctions cup's solenoid from being stuck in indeterminant state
 */
public class CheckSucc extends CommandBase {
    public CheckSucc() {
        addRequirements(Robot.intake);
    }
    
   public void initialize() {
        setTimeout(0.2);
        Robot.intake.releaseSolenoid(true);
    }

    public boolean isFinished() {
        return isTimedOut();
    }

    public void end(boolean interrupted) {
        Robot.intake.releaseSolenoid(false);
    }
}