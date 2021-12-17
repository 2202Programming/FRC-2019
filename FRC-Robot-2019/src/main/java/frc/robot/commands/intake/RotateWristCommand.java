package frc.robot.commands.intake; 

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;


/**
 * 
 * Use a timeout to wait, so a "AND" test is performed because
 * the actual servo has now way of reporting back to Rio that
 * we aren't at the desired postion.  Time out gives it time to get there.
 * 
 */
public class RotateWristCommand extends WaitCommand{
    private double angle;

    public RotateWristCommand(double angle, double timeout){
        super(timeout);
        addRequirements(Robot.intake);
        this.angle = angle;
    }

    public RotateWristCommand(double angle) {
        this(angle, 0.0);
    }

  @Override
  public void execute() {
      Robot.intake.setAngle(angle);
  }

  @Override
  public boolean isFinished() {
     // position in 1.0 degrees, but servo will always hit.
    double measAngle = Robot.intake.getAngle();
    boolean posHit = ( Math.abs(measAngle - angle) < 1.0 );
     //stay for the whole timeout
     boolean to = super.isFinished();
     return (to && posHit);
  }  
}

