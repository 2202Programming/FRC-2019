package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.util.RateLimiter;
import frc.robot.commands.util.RateLimiter.InputModel;

public class ClimbRollForward extends Command {
    private double timeout;
    private double rollerSpeed;
    private double curSpeed;
    //private RateLimiter sLimit;
    private double limitFactor = 10.0;

    public ClimbRollForward(double rollerSpeed, double timeout) {
        this.timeout = timeout;
        this.rollerSpeed = rollerSpeed;
        requires(Robot.climber);
        //sLimit = new RateLimiter(Robot.dT, this::getRollerSpeed, null, 0, this.rollerSpeed, 0, (rollerSpeed/limitFactor), InputModel.Position);
    }

    private double getRollerSpeed() {
        return rollerSpeed;
    }

    protected void initialize() {
        setTimeout(timeout);
        Robot.climber.setRollerSpeed(0);
        curSpeed = 0;
    }

    protected void execute() {
        curSpeed += rollerSpeed / limitFactor;
        if (curSpeed > rollerSpeed) curSpeed = rollerSpeed;
        //sLimit.execute();
        //double speed = sLimit.get();
        Robot.climber.setRollerSpeed(curSpeed);
    }

    protected void end() {
        Robot.climber.setRollerSpeed(0);
    }

    protected boolean isFinished() {
        return isTimedOut();
    }
}