package frc.robot.commands.arm;

import frc.robot.commands.util.MathUtil;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/** 
 * 
 *   Billy & X think this is working for forward facing arm.
 * 
 *   TODO:FIX this to take account for which side arm is on
 * 
 *   2/20/19   DPL   adjusted to take h and l commands from functions
 *                   cleaned up constraints
 */

public class MoveArmAtHeight extends Command {
    //ArmSubsystem  arm = Robot.arm;

    // Length of the arm from pivot point without extension in inches
    private final double armInitialLength = Robot.arm.EXTEND_MIN + Robot.arm.ARM_BASE_LENGTH
            + Robot.intake.WristDistToPivot + Robot.arm.L0;

    // Height of point of rotation for the arm in inches
    private final double pivotHeight = Robot.arm.ARM_PIVOT_HEIGHT;

    // Make an h' to more easily construct a triangle, relative to pivot height
    private double h;

    // Projection of the arm on the ground
    private double xProjection;

    //Inputs to this Command
    DoubleSupplier extCmdFunct;
    DoubleSupplier heightCmdFunct;

    public MoveArmAtHeight(DoubleSupplier heightCmdFunct, DoubleSupplier extCmdFunct) {
        requires(Robot.arm);
        this.heightCmdFunct = heightCmdFunct;
        this.extCmdFunct = extCmdFunct;
    }

    /**
     * This only works for front side of robot...
     */

    protected void execute() {
        // read inputs
        double h_cmd = heightCmdFunct.getAsDouble(); //height above floor inches
        double l_cmd = extCmdFunct.getAsDouble();    //length from from pivot

        // If target is below pivot height, used for quadrant calcs
        boolean belowPiv = h_cmd < pivotHeight;
        h = Math.abs(pivotHeight - h_cmd);           //h (inches) above or below piviot in mag
        
        //Roughly limit the extension based on game limits and robot geometry
        xProjection = MathUtil.limit(l_cmd, Robot.arm.getInversionConstant() * Robot.arm.MIN_PROJECTION,
                                            Robot.arm.getInversionConstant() * Robot.arm.MAX_PROJECTION);

        double tanRatio = (belowPiv) ? h / xProjection : xProjection / h;
        // Rotate to maintain height as projection changes
        double angle = Math.toDegrees(Math.atan(tanRatio));
        if (Robot.arm.getInversionConstant() > 0) angle += (belowPiv) ? 90.0 : 0.0;
        else angle -= (belowPiv) ? 90.0 : 0.0;

        angle = MathUtil.limit(angle, Robot.arm.PHI_MIN, Robot.arm.PHI_MAX); 
        
        double projLen= Math.sqrt( h*h + xProjection * xProjection);    //total length of arm, from pivot point
        double ext  = projLen - (Robot.arm.ARM_BASE_LENGTH + Robot.arm.WRIST_LENGTH);   // extension required
        
        double compLen = Robot.arm.getCompLen(angle);

        //limit within range, TODO: do we need to account for phi/ext interaction here?
        ext = MathUtil.limit(ext, Robot.arm.EXTEND_MIN, Robot.arm.EXTEND_MAX);

        // Extend to allow for change in projection
        Robot.arm.setExtension(ext);   //absolute ext needed projection
        Robot.arm.setAngle(angle);     //angle required for height
        /*
         * Alternative extension calculation xProjection /
         * Math.cos(Robot.arm.getAngle()) - armInitialLength
         */
    }

    protected boolean isFinished() {
        return false;
    }

}