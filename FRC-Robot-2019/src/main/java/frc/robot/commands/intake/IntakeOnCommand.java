package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class IntakeOnCommand extends InstantCommand {

    public IntakeOnCommand() {
        super("vacuum-on");
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        Robot.intake.vacuumOn();
    }
}
