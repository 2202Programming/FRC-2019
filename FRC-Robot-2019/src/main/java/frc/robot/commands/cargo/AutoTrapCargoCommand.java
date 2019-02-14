package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class AutoTrapCargoCommand extends CommandGroup {
    public AutoTrapCargoCommand() {
        addSequential(new DeployCargoTrapCommand());
        addSequential(new CloseCargoTrapCommand());
    }
}