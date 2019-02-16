package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class OuttakeCommand extends InstantCommand {

    public OuttakeCommand() {
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        Robot.intake.vacuumOff();    
    }
}
