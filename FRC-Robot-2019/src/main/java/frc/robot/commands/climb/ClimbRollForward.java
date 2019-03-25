package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbRollForward extends Command {
    private double timeout;
    private double rollerSpeed;

    public ClimbRollForward(double rollerSpeed, double timeout) {
        this.timeout = timeout;
        this.rollerSpeed = rollerSpeed;
        requires(Robot.climber);
    }

    protected void initialize() {
        setTimeout(timeout);
        Robot.climber.setRollerSpeed(rollerSpeed);
    }

    protected void execute() {
        
    }

    protected void end() {
        Robot.climber.setRollerSpeed(0);
    }

    protected boolean isFinished() {
        return isTimedOut();
    }
}