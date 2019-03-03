package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbRollForward extends Command {
    public ClimbRollForward() {
        requires(Robot.climber);
    }

    protected void execute() {
        Robot.climber.setRollerSpeed(0.5);
        Robot.climber.setDrawerSlide(true); //activates the drawer slide piston
    }

    protected void end() {
        Robot.climber.setRollerSpeed(0);
        Robot.climber.setDrawerSlide(false);
    }

    protected boolean isFinished() {
        return isTimedOut();
    }
}