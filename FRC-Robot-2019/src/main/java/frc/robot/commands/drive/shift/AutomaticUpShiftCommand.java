package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;

public class AutomaticUpShiftCommand extends CommandGroup {
    public AutomaticUpShiftCommand() {
        addParallel(new UpShiftCommand());
        addParallel(new ThrottleCommand(1.0, 0.5, 1));
    }
}