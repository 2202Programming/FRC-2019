package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class IntakeCommand extends InstantCommand {

    public IntakeCommand() {
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        Robot.intake.vacuumOn();
    }
}
