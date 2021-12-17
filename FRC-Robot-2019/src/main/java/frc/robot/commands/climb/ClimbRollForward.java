package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ClimbRollForward extends CommandBase {
    private double rollerSpeed;
    private double curSpeed;
    private double startSpeed;
    private double acceleration;

    public ClimbRollForward(double startSpeed, double rollerSpeed, double powerAcceleration) {
        addRequirements(Robot.climber);
        this.startSpeed = startSpeed;
        this.rollerSpeed = rollerSpeed;
        this.acceleration = powerAcceleration * Robot.kDefaultPeriod;
    }

   public void initialize() {
        Robot.climber.setRollerSpeed(0);
        curSpeed = startSpeed;
    }

    public void execute() {
        if(rollerSpeed > 0) {
            curSpeed = Math.min(rollerSpeed, curSpeed + acceleration);
        } else {
            Math.max(rollerSpeed, curSpeed - acceleration);
        }
        Robot.climber.setRollerSpeed(curSpeed);
    }

    public void end(boolean interrupted) {
        Robot.climber.setRollerSpeed(0);
    }

    public boolean isFinished() {
        return Robot.climber.climberAgainstWall();
    }
}