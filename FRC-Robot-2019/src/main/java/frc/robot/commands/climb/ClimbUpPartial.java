package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.MoveArmToRawPosition;

public class ClimbUpPartial extends SequentialCommandGroup {
    public ClimbUpPartial(double climbHeight, double retractHeight) {

        addCommands(Robot.climber.zeroSubsystem(),    //hack to zero counters
            new MoveArmToRawPosition(90, 29, 0.5, 20),
            new PawlSureFire(Robot.climber.Extend, 4),
            new DeployClimbFoot(0.9, climbHeight)
        );    // 20.5 uses limit switch
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
    public void end(boolean interrupted) {
        Robot.driveTrain.stop();
        Robot.climber.setRollerSpeed(0.0);
        Robot.climber.setExtenderSpeed(0.0);
        Robot.climber.setPawl(Robot.climber.Retract);
        Robot.climber.setDrawerSlide(Robot.climber.HoldSlide);
    }
}