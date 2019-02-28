package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbGroup extends CommandGroup {
    public ClimbGroup() {
        addSequential(new DeployClimbFoot());
        addParallel(new ClimbRollForward());
        addSequential(new RetractClimbFoot());
    }
}