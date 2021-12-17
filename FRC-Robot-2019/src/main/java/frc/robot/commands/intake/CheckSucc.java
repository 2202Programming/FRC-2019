package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

/**
 * Stops the suctions cup's solenoid from being stuck in indeterminant state
 */
public class CheckSucc extends WaitCommand {
    public CheckSucc() {
        super(.2);
        addRequirements(Robot.intake);
    }
    @Override
    public void initialize() {
        super.initialize();
        Robot.intake.releaseSolenoid(true);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Robot.intake.releaseSolenoid(false);
    }
}