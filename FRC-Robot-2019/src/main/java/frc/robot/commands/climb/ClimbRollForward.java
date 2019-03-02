package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbRollForward extends Command {
    public ClimbRollForward() {
        requires(Robot.climber);
    }

    protected void execute() {
        Robot.climber.setRollerSpeed(0.5);
    }

    protected void end() {
        Robot.climber.setRollerSpeed(0);
    }

    protected boolean isFinished() {
        return false; //TODO: Find good way to stop moving forward (Timer for macro)
    }
}