package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbRollForward extends Command {
    private double timeout;

    public ClimbRollForward(double timeout) {
        this.timeout = timeout;
        requires(Robot.climber);
    }

    public ClimbRollForward() {
        timeout = 999;
        requires(Robot.climber);
    }

    protected void initialize() {
        setTimeout(timeout);
        Robot.climber.setDrawerSlide(true); //activates the drawer slide piston
    }

    protected void execute() {
        Robot.climber.setRollerSpeed(0.5);
    }

    protected void end() {
        Robot.climber.setRollerSpeed(0);
        Robot.climber.setDrawerSlide(false);
    }

    protected boolean isFinished() {
        return isTimedOut();
    }
}