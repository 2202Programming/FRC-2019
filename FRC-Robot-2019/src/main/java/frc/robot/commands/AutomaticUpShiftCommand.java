package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;

public class AutomaticUpShiftCommand extends CommandGroup {
    public AutomaticUpShiftCommand() {
        requires(Robot.gearShifter);
        requires(Robot.driveTrain);

        CommandGroup upShiftParallel = new CommandGroup();
        upShiftParallel.addSequential(new WaitCommand(0.1));
        upShiftParallel.addSequential(new UpShiftCommand());
        addParallel(upShiftParallel);
        addSequential(new ThrottleCommand(2, 0.5, 1));
    }
}