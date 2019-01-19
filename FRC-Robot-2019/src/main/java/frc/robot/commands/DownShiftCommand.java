package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class DownShiftCommand extends InstantCommand {
    public DownShiftCommand() {
        requires(Robot.gearShifter);
    }

    public void initialize() {
        Robot.gearShifter.shiftDown();
    }
}