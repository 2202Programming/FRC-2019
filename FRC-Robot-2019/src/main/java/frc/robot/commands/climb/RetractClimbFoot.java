package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RetractClimbFoot extends Command {
    public RetractClimbFoot() {
        requires(Robot.climber);
    }

    protected void execute() {
        Robot.climber.setExtenderSpeed(-0.3);
    }

    protected void end() {
        Robot.climber.setExtenderSpeed(0);
    }

    protected boolean isFinished() {
        return Robot.climber.getExtension() <= 10; //TODO: Better stop at bottom
    }
}