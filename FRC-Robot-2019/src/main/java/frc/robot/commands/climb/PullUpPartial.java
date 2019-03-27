package frc.robot.commands.climb;

import frc.robot.commands.drive.DriveByPowerAndJoystickCommand;
import frc.robot.commands.drive.DriveByPowerCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.CallFunctionCommand;
import frc.robot.Robot;
import frc.robot.commands.GripperPositionCommand;
import frc.robot.commands.arm.FlipCommand;
import frc.robot.commands.arm.MoveArmAtHeight;

public class PullUpPartial extends CommandGroup {
    public PullUpPartial(double climbHeight, double retractHeight) {
        double drivePower = 0.35; // Positive power goes to negative direction

        addSequential(new FlipCommand(90, -90, 12, 0.5, 20));
        CommandGroup forwardCmds3 = new CommandGroup("going forward 3");
        forwardCmds3.addSequential(new PawlSureFire(Robot.climber.Retract,  5));
        forwardCmds3.addParallel(new DeployClimbFoot(-0.50, retractHeight));    // neg power retract / limit sw
        forwardCmds3.addParallel(new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6, 3.0)); // neg power drive reverse
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