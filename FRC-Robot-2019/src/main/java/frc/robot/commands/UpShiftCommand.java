package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class UpShiftCommand extends InstantCommand {
    public UpShiftCommand() {
        requires(Robot.gearShifter);
    }

    @Override
    protected void execute() {
        Robot.gearShifter.shiftUp();
    }
}