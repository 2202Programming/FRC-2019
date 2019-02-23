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
    
    private final double kTolerance = 1.0;

    //Projection of the arm on the ground to be maintained
    private double curProjection;

    private double curCalcHeight;
    private double endHeight;
    private double down_cmd;   //inches to move down for pickup

    public MoveDownToCapture(double down){
        requires(Robot.arm);
        down_cmd = down;

        
    }

    protected void initialize() {
        curProjection = (armInitialLength + Robot.arm.getExtension()) * Math.cos(Robot.arm.getAngle());
        curCalcHeight = Math.sqrt((Robot.arm.getExtension() + armInitialLength) * (Robot.arm.getExtension() + armInitialLength) - curProjection * curProjection);
        endHeight = curCalcHeight + down_cmd; //Move down 5 inches (increase bc increasing distance from x-axis)
        /*
        Alternative method of calculating height
        
        */
    }

    // A button will trigger this when the pilot expects to be over a hatch/cargo.
    // We need to move into capturing mode, move down. wait a bit
    protected void execute() {
        double angle = 90 + Math.toDegrees(Math.atan(endHeight / curProjection));
        double extension = Math.sqrt(endHeight * endHeight + curProjection * curProjection) - armInitialLength;
        //Move to the endHeight while maintaining curProjection
        Robot.arm.setAngle(angle);
        //Add 90 bc calc goes below x axis

        Robot.arm.setExtension(extension);

    }

    protected boolean isFinished() {
        return Math.abs(curCalcHeight - endHeight) < kTolerance;
    }
}