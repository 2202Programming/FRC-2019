package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbRollForward extends Command {
    private double rollerSpeed;
    private double curSpeed;
    private double startSpeed;
    private double acceleration;

    public ClimbRollForward(double startSpeed, double rollerSpeed, double powerAcceleration) {
        requires(Robot.climber);
        this.startSpeed = startSpeed;
        this.rollerSpeed = rollerSpeed;
        this.acceleration = powerAcceleration * Robot.kDefaultPeriod;
    }

    protected void initialize() {
        Robot.climber.setRollerSpeed(0);
        curSpeed = startSpeed;
    }

    protected void execute() {
        if(rollerSpeed > 0) {
            curSpeed = Math.min(rollerSpeed, curSpeed + acceleration);
        } else {
            Math.max(rollerSpeed, curSpeed - acceleration);
        }
        Robot.climber.setRollerSpeed(curSpeed);
    }

    protected void end() {
        Robot.climber.setRollerSpeed(0);
    }

    protected boolean isFinished() {
        return Robot.climber.climberAgainstWall();
    }
}