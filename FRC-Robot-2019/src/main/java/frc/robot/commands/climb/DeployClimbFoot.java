package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DeployClimbFoot extends Command {
    public DeployClimbFoot() {
        requires(Robot.climber);
    }

    protected void execute() {
        Robot.climber.setRatchetExtend(true);
        Robot.climber.setExtenderSpeed(0.3);
    }

    protected void end() {
        Robot.climber.setExtenderSpeed(0);
    }

    protected boolean isFinished() {
        int x = 1; //will be value for 19 inches
        return Robot.climber.getExtentionEncoder().get() >= x; //TODO: stop when it extends 19 inches
    }
}