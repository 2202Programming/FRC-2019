package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class UpShiftCommand extends InstantCommand {
    public UpShiftCommand() {
        addRequirements(Robot.gearShifter);
    }

    @Override
    public void execute() {
        Robot.gearShifter.shiftUp();
    }
}