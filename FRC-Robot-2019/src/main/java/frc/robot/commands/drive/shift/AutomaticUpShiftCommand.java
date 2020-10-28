package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class AutomaticUpShiftCommand extends CommandGroup {
    public AutomaticUpShiftCommand() {
        addParallel(new UpShiftCommand());
        addParallel(new ThrottleCommand(1.0, 0.5, 1));
    }
}