package frc.robot.commands.arm;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveArmAtHeight extends Command {
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

    private boolean belowX;

    public MoveArmAtHeight(double height){
        requires(Robot.arm);
        belowX = height < pivotHeight;
        if (!belowX) calculationHeight = height - pivotHeight;
        else calculationHeight = pivotHeight - height;
    }

    protected void execute() {
        //TODO: Mapping joystick properly to change in projection
        xProjection = projectionInitialLength + Robot.m_oi.getAssistantController().getY();

        if (xProjection > projectionMax) xProjection = projectionMax;

        //Rotate to maintain height as projection changes
        if (!belowX) Robot.arm.setAngle(Math.toDegrees(Math.atan(calculationHeight / xProjection)));
        else Robot.arm.setAngle(90 + Math.toDegrees(Math.atan(calculationHeight / xProjection)));

        //Extend to allow for change in projection
        Robot.arm.setExtension(
            Math.sqrt(calculationHeight * calculationHeight + xProjection * xProjection) - armInitialLength);

        /*Alternative extension calculation
        xProjection / Math.cos(Robot.arm.getAngle()) - armInitialLength */
        
    }

    protected boolean isFinished() {
        return false;
    }
}