package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.MoveArmToRawPosition;
import frc.robot.commands.drive.DriveByPowerAndJoystickCommand;
import frc.robot.commands.drive.HABDriveByPowerAndJoystickCommand;

public class ClimbGroup extends CommandGroup {
    public ClimbGroup(double climbHeight, double retractHeight) {
        double timeToDriveForward = 3.0;
        double rollPower = 0.5;
        double drivePower = 0.4; // Positive power goes to negative direction

        //if separate command to bring up robot change to parallel
        addSequential(Robot.climber.zeroSubsystem());   //hack to zero counters
        addSequential(new MoveArmToRawPosition(90, 12, 0.5, 180));        
        addSequential(new PawlSureFire(Robot.climber.Extend, 4));
        addSequential(new DeployClimbFoot(0.9, climbHeight));    // 20.5 uses limit switch

        CommandGroup forwardCmds = new CommandGroup("Going forward");
        forwardCmds.addParallel(new ClimbRollForward(rollPower));   // power, timeout
        forwardCmds.addParallel(new HABDriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6)); // power, timeout

        addSequential(forwardCmds);
        addSequential(new MoveArmToRawPosition(-90, 9, 0.6, 180));
        addSequential(new WaitCommand(3));
        CommandGroup forwardCmds3 = new CommandGroup("Going forward 2");
        forwardCmds3.addSequential(new PawlSureFire(Robot.climber.Retract,  6));
        forwardCmds3.addParallel(new DeployClimbFoot(-0.50, retractHeight));    // neg power retract / limit sw
        forwardCmds3.addParallel(new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6, timeToDriveForward)); // neg power drive reverse
        addSequential(forwardCmds3);
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

    @Override
    protected void interrupted() {
        Robot.driveTrain.stop();
        Robot.climber.setRollerSpeed(0.0);
        Robot.climber.setExtenderSpeed(0.0);
        Robot.climber.setPawl(Robot.climber.Retract);
        Robot.climber.setDrawerSlide(Robot.climber.HoldSlide);
    }
}