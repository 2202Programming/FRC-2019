package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class DownShiftCommand extends InstantCommand {
    public DownShiftCommand() {
        requires(Robot.gearShifter);
    }

    @Override
    protected void execute() {
        Robot.gearShifter.shiftDown();
    }
}