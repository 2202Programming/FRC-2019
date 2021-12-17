package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 *    distance in inches, use large number to rely on limit swithc
 */
public class DeployClimbFoot extends CommandBase {
    double power;
    double distance;
    boolean extending;

    public DeployClimbFoot(double power, double distance) {
        addRequirements(Robot.climber);
        this.power = power;
        this.distance = distance;
        extending = (power > 0);    //confirmed pos power is extending 
    }

    @Override
   public void initialize() {
        Robot.climber.setExtenderSpeed(power);
    }

    public void execute() {  }

    public void end(boolean interrupted) {
        Robot.climber.setExtenderSpeed(0);
    }

    public boolean isFinished() {
        // depending on direction (power indicates)
        boolean done;
        // perform check based on which way we are  moving the foot
        done = (extending) ? ((Robot.climber.getExtension() >= distance) || Robot.climber.footAtExtend() )
                           : ((Robot.climber.getExtension() <= distance) || Robot.climber.footAtRetract() );
        return done; 
    }
}