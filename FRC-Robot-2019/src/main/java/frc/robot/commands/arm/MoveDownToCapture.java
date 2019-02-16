package frc.robot.commands.arm;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveDownToCapture extends Command {
    /*
    Constants should probably be moved to the subsystem?
    Many commands will be involved in moving the arm in an x-y coordinate system
    They will all require these constants to some extent
    */

    //Length of the arm from pivot point without extension in inches
    private final double armInitialLength = 30.0;
    //Height of point of rotation for the arm in inches
    private final double pivotHeight = 29.75;
    
    private final double kTolerance = 1.0;

    //Make an h' to more easily construct a triangle
    private final double calculationHeight;

    //Projection of the arm on the ground to be maintained
    private double curProjection;

    private double curHeight;
    private double endHeight;

    public MoveDownToCapture(){
        requires(Robot.arm);
        curProjection = (armInitialLength + Robot.arm.getExtension()) * Math.cos(Robot.arm.getAngle());
        curHeight = Math.sqrt(Robot.arm.getExtension() * Robot.arm.getExtension() - curProjection * curProjection);
        endHeight = curHeight - 5.0; //Move down 5 inches
        calculationHeight = pivotHeight - endHeight;
        /*
        Alternative method of calculating height
        
        */
    }

    protected void execute() {
        curHeight = Math.sqrt(Robot.arm.getExtension() * Robot.arm.getExtension() - curProjection * curProjection);

        //Move to the endHeight while maintaining curProjection
        Robot.arm.setAngle(90 + Math.toDegrees(Math.atan(calculationHeight / curProjection)));
        //Add 90 bc calc goes below x axis

        Robot.arm.setExtension(
            Math.sqrt(calculationHeight * calculationHeight + curProjection * curProjection) - armInitialLength);
    }

    protected boolean isFinished() {
        return Math.abs(curHeight - endHeight) < kTolerance;
    }
}