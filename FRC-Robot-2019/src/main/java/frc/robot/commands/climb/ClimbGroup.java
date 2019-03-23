package frc.robot.commands.climb;

import frc.robot.commands.drive.DriveByPowerCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.CallFunctionCommand;
import frc.robot.Robot;
import frc.robot.commands.GripperPositionCommand;

public class ClimbGroup extends CommandGroup {
    public ClimbGroup() {
        double longTO = 5.0;
        CommandGroup armGrp = new CommandGroup();
        //move the arm to the back side
        armGrp.addSequential(new GripperPositionCommand(64.0, 1.0, 0.05, longTO)); 
        armGrp.addSequential(new CallFunctionCommand(Robot.arm::invert));
        armGrp.addSequential(new GripperPositionCommand(64.0, 5.0, 0.05, longTO));
        armGrp.addSequential(new GripperPositionCommand(25.0, 40.0, 0.05, longTO));

        addSequential(armGrp);

        //if separate command to bring up robot change to parallel
        addSequential(new PawlSureFire(Robot.climber.Release, 1));
        addSequential(new DeployClimbFoot(0.85, 19.0));
        //go forward while driving foot
        CommandGroup forwardCmds = new CommandGroup("going forward");
        forwardCmds.addParallel(new ClimbRollForward(0.6, 10.0 ));  // power, timeout
        forwardCmds.addParallel(new DriveByPowerCommand(-.15, 5.0)); // power, timeout
        addSequential(forwardCmds);
        addSequential(new CallFunctionCommand(this::setCharon));
        CommandGroup forwardCmds2 = new CommandGroup("going forward 2");
        forwardCmds2.addSequential(new PawlSureFire(Robot.climber.PullIn,  1));
        forwardCmds2.addParallel(new DeployClimbFoot(-0.6, 0.33));   // neg power retract 
        forwardCmds2.addParallel(new DriveByPowerCommand(-.15, 2.0)); // neg power drive reverse
        addSequential(forwardCmds2);
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

    private int setCharon() {
        Robot.climber.setDrawerSlide(true);
        return 0;
    }

}