package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *    distance in inches, use large number to rely on limit swithc
 */
public class DeployClimbFoot extends Command {
    double power;
    double distance;
    boolean extending;

    public DeployClimbFoot(double power, double distance) {
        requires(Robot.climber);
        this.power = power;
        this.distance = distance;
    }

    @Override
    protected void initialize() {
        Robot.climber.setExtenderSpeed(power);
        extending = (power > 0);    //confirmed pos power is extending
    }

    protected void execute() {  }

    protected void end() {
        Robot.climber.setExtenderSpeed(0);
    }

    protected boolean isFinished() {
        // depending on direction (power indicates)
        boolean done;
        // perform check based on which way we are  moving the foot
        done = (extending) ? ((Robot.climber.getExtension() >= distance) || Robot.climber.extensionAtMax() )
                           : ((Robot.climber.getExtension() <= distance) || Robot.climber.extensionAtMin() );
        return done; 
    }
}