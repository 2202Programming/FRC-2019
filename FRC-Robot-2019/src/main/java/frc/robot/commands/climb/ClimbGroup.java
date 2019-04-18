package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.CommandManager.Modes;
import frc.robot.commands.arm.MoveArmToRawPosition;
import frc.robot.commands.drive.DriveByPowerAndJoystickCommand;
import frc.robot.commands.drive.HABDriveByPowerAndJoystickCommand;
import frc.robot.commands.intake.WristSetAngleCommand;
import frc.robot.commands.intake.WristTrackAngle;
import frc.robot.commands.util.Angle;

public class ClimbGroup extends CommandGroup {
    public ClimbGroup(double climbHeight, double retractHeight) {
        double timeToDriveForward = 30.0;
        double rollPower = 0.6;
        double drivePower = 0.4; // Positive power goes to negative direction

        //if separate command to bring up robot change to parallel
        addSequential(Robot.climber.zeroSubsystem());   //hack to zero counters
        addSequential(new WristSetAngleCommand(0));
        addSequential(new MoveArmToRawPosition(90, 12, 0.5, 180));        
        addSequential(new PawlSureFire(Robot.climber.Extend, 4));
        addSequential(new DeployClimbFoot(1.0, climbHeight));    // 20.5 uses limit switch
        addSequential(new WaitCommand(0.5));

        CommandGroup forwardCmds = new CommandGroup("Going forward");
        forwardCmds.addParallel(new ClimbRollForward(0.2, rollPower, 0.3));   // power, timeout
        forwardCmds.addParallel(new HABDriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6)); // power, timeout
        
        addSequential(forwardCmds);
        CommandGroup forwardCmds3 = new CommandGroup("Going forward 2");
        forwardCmds3.addSequential(new PawlSureFire(Robot.climber.Retract,  6));
        forwardCmds3.addParallel(new DeployClimbFoot(-1.0, retractHeight));    // neg power retract / limit sw
        forwardCmds3.addParallel(new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6, timeToDriveForward)); // neg power drive reverse
        
        addSequential(new MoveArmToRawPosition(-90, 12, 0.6, 180));
        addSequential(forwardCmds3);

        addSequential(new MoveArmToRawPosition(-90, 6, 0.6, 180));
        addParallel(new WristTrackAngle(Angle.Back_Perpendicular_Down.getAngle()));
        addSequential(new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6, 200.0));
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
        Robot.m_cmdMgr.setMode(Modes.Drive);
    }

    protected void end() {

    }
}