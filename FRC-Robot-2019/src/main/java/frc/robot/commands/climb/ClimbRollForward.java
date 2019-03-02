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
        Robot.climber.setDrawerSlide(Value.kForward); //activates the drawer slide piston
    }

    protected void end() {
        Robot.climber.setRollerSpeed(0);
        Robot.climber.setDrawerSlide(Value.kOff);
    }

    protected boolean isFinished() {
        return false; //TODO: Find good way to stop moving forward (Timer for macro)
    }
}