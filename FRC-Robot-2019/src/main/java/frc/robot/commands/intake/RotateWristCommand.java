package frc.robot.commands.intake; 

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


/**
 * 
 * Use a timeout to wait, so a "AND" test is performed because
 * the actual servo has now way of reporting back to Rio that
 * we aren't at the desired postion.  Time out gives it time to get there.
 * 
 */
public class RotateWristCommand extends Command{
    private double angle;
    private double timeout;

    public RotateWristCommand(double angle, double timeout){
        requires(Robot.intake);
        this.angle = angle;
        this.timeout = timeout;
    }

    public RotateWristCommand(double angle) {
        this(angle, 0.0);
    }

    @Override
    protected void initialize() {
        setTimeout(timeout);
    }

  @Override
  protected void execute() {
      Robot.intake.setAngle(angle);
  }

  @Override
  protected boolean isFinished() {
     // position in 1.0 degrees, but servo will always hit.
    double measAngle = Robot.intake.getAngle();
    boolean posHit = ( Math.abs(measAngle - angle) < 1.0 );
     //stay for the whole timeout
     boolean to = isTimedOut();
     return (to && posHit);
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
      return;
  }
}

