package frc.robot.commands.climb;

import frc.robot.commands.drive.DriveByPowerAndJoystickCommand;
import frc.robot.commands.drive.DriveByPowerCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.CallFunctionCommand;
import frc.robot.Robot;
import frc.robot.commands.GripperPositionCommand;

public class ClimbGroup extends CommandGroup {
    public ClimbGroup(double climbHeight) {
        double longTO = 5.0;
        double timeToDriveForward = 2.0;
        double rollPower = 0.3;
        double drivePower = 0.3; // Positive power goes to negative direction

        CommandGroup armGrp = new CommandGroup();
        //move the arm to the back side
        armGrp.addSequential(new GripperPositionCommand(64.0, 1.0, 0.05, longTO)); 
        armGrp.addSequential(new CallFunctionCommand(Robot.arm::invert));
        armGrp.addSequential(new GripperPositionCommand(64.0, 5.0, 0.05, longTO));
        armGrp.addSequential(new GripperPositionCommand(25.0, 40.0, 0.05, longTO));

        //addSequential(armGrp);

        //if separate command to bring up robot change to parallel
        addSequential(Robot.climber.zeroSubsystem());   //hack to zero counters
        addSequential(new PawlSureFire(Robot.climber.Extend, 4));
        addSequential(new DeployClimbFoot(0.9, climbHeight));    // 20.5 uses limit switch
        //go forward while driving foot
        CommandGroup forwardCmds = new CommandGroup("going forward1");
        forwardCmds.addParallel(new ClimbRollForward(rollPower, timeToDriveForward ));   // power, timeout
        forwardCmds.addParallel(new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.5, timeToDriveForward)); // power, timeout
        
        addSequential(forwardCmds);
        addSequential(new WaitCommand(0.2));
        addSequential(new CallFunctionCommand(this::releaseSlide));

        timeToDriveForward = 2.0;
        CommandGroup forwardCmds2 = new CommandGroup("going forward2");
        forwardCmds2.addParallel(new ClimbRollForward(rollPower, timeToDriveForward ));   // power, timeout
        forwardCmds2.addParallel(new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.5, timeToDriveForward)); // power, timeout

        addSequential(forwardCmds2);

        CommandGroup forwardCmds3 = new CommandGroup("going forward 3");
        forwardCmds3.addSequential(new PawlSureFire(Robot.climber.Retract,  4));
        forwardCmds3.addParallel(new DeployClimbFoot(-0.50, 0.0));    // neg power retract / limit sw
        forwardCmds3.addParallel(new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.5, 3.0)); // neg power drive reverse
        addSequential(forwardCmds3);
        addSequential(new CallFunctionCommand(this::holdSlide));
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

    private int releaseSlide() {
        Robot.climber.setDrawerSlide(Robot.climber.ReleaseSlide);
        return 0;
    }

    private int holdSlide() {

        Robot.climber.setDrawerSlide(Robot.climber.HoldSlide);
        return 0;
    }

    @Override
    protected void interrupted() {
        Robot.driveTrain.stop();
        Robot.climber.setRollerSpeed(0.0);
        Robot.climber.setExtenderSpeed(0.0);
        Robot.climber.setPawl(Robot.climber.Retract);
        Robot.climber.setDrawerSlide(Robot.climber.HoldSlide);
    }
}