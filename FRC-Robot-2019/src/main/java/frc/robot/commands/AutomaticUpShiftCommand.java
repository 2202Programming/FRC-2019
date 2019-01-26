package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;

public class AutomaticUpShiftCommand extends CommandGroup {
    public AutomaticUpShiftCommand() {
        requires(Robot.gearShifter);
        requires(Robot.driveTrain);

        CommandGroup upShiftParallel = new CommandGroup();
        upShiftParallel.addSequential(new WaitCommand(0.25));
        upShiftParallel.addSequential(new UpShiftCommand());
        addSequential(new ThrottleCommand(1, 0.5, 1));
        addParallel(upShiftParallel);
    }
}