package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbGroup extends CommandGroup {
    public ClimbGroup() {
        //if separate command to bring up robot change to parallel
        addSequential(new PawlSureFire(true));
        addSequential(new DeployClimbFoot());
        addSequential(new ClimbRollForward());
        addSequential(new PawlSureFire(false));
        addSequential(new RetractClimbFoot());
    }

    /*
    steps for climb: * - means I attemped but probably needs more
    driver drives back into hab
    engage pawl piston - keep engaged *
    run m23 until we reach 19 inches - there is an encoder *
    stop m23 *
    engage drawer slide piston *
    run drive wheels - crawling backwards
    run m22 - open loop until drawer slides in *
    disengage drawer slide piston *
    stop drive wheels
    stop m22 *
    disengage pawl piston *
    run m23 backwards until climber is mostly up *
    end

    pawl piston is foot, m23 is foot extension motor, m22 is motor to move foot
    */
}