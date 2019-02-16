package frc.robot.commands.arm;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveArmAtHeight extends Command {
    //Length of the arm from pivot point without extension in inches
    private final double armInitialLength = Robot.arm.EXTEND_MIN + Robot.arm.ARM_BASE_LENGTH;
    //Height of point of rotation for the arm in inches
    private final double pivotHeight = Robot.arm.ARM_PIVOT_HEIGHT;
    
    /*TODO: Legit length
    Starting projection of arm (starts at edge of bumper) in inches */
    private final double projectionInitialLength = 16;   //TODO:FIX this to take account for which side arm is on
    // there will be two limit - forward side and rear side.  - Derek/Shawn  

    DoubleSupplier getHeight;

    //Maximum projection based on 
    private final double projectionMax = projectionInitialLength + Robot.kProjectConstraint;

    //Make an h' to more easily construct a triangle
    private final double calculationHeight;

    //Projection of the arm on the ground
    private double xProjection;

    private boolean belowX;

    public MoveArmAtHeight(DoubleSupplier heightFunct){
        requires(Robot.arm);
        double height = heightFunct.getAsDouble();

        belowX = height < pivotHeight;
        if (!belowX) calculationHeight = height - pivotHeight;
        else calculationHeight = pivotHeight - height;
        xProjection = projectionInitialLength;
    }

    protected void execute() {
        double projChange = Robot.m_oi.getAssistantController().getY();
        //TODO: Mapping joystick properly to change in projection
        xProjection += projChange;

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