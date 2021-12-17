package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DownShiftCommand extends InstantCommand {
    public DownShiftCommand() {
        addRequirements(Robot.gearShifter);
    }

    @Override
    public void execute() {
        Robot.gearShifter.shiftDown();
    }
}