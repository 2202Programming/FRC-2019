package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeCommand extends Command {

    public IntakeCommand() {
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        Robot.intake.vacuumOn();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
