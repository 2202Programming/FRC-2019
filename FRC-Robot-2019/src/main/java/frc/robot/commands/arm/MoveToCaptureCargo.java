package frc.robot.commands.arm;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveToCaptureCargo extends Command {
    /*
    Constants should probably be moved to the subsystem?
    Many commands will be involved in moving the arm in an x-y coordinate system
    They will all require these constants to some extent
    */

    //Length of the arm from pivot point without extension in inches
    private final double armInitialLength = 30;
    //Height of point of rotation for the arm in inches
    private final double pivotHeight = 29.75;
    
    /*TODO: Legit length
    Starting projection of arm (starts at edge of bumper) in inches */
    private final double projectionInitialLength = 16;

    //Maximum projection based on 
    private final double projectionMax = projectionInitialLength + 30;

    //Make an h' to more easily construct a triangle
    private final double calculationHeight;

    //Projection of the arm on the ground
    private double xProjection;

    public MoveToCaptureCargo(){
        requires(Robot.arm);
    }

    protected void execute() {
        
    }

    protected boolean isFinished() {
        return false;
    }
}