package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class StopWristCommand extends InstantCommand {

    public StopWristCommand() {
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        Robot.intake.stopWrist();
    }
}