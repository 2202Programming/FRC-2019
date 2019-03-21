package frc.robot.commands.arm;

import frc.robot.commands.CommandManager.Modes;
import frc.robot.commands.util.ExpoShaper;
import frc.robot.commands.util.LimitedIntegrator;
import frc.robot.commands.util.MathUtil;
import frc.robot.commands.util.RateLimiter;
import frc.robot.commands.util.RateLimiter.InputModel;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/** 
 * 
 *   Billy & X think this is working for forward facing arm.
 * 
 *   2/20/19   DPL   adjusted to take h and l commands from functions
 *                   cleaned up constraints
 */

public class MoveAtHeightDefault extends Command {

    private ArmSubsystem arm;
    // Height of point of rotation for the arm in inches
    private final double pivotHeight = Robot.arm.ARM_PIVOT_HEIGHT;
    public final double kCapHeight = 4.0; // inch/joy units TODO: put in better place

    // Make an h' to more easily construct a triangle, relative to pivot height
    private double h;

    // Projection of the arm on the ground
    private double xProjection;

    final double DeliveryCargoHeights[] = { 26.875, 55.0, 84.0 }; // TODO: fix the numbers
    final double DeliveryHatchHeights[] = { 27.5, 55.0, 82.0 }; 
    final double deliveryProjection[] = { 25.0, 25.0, 25.0 }; // TODO: fix the numbers

    private LimitedIntegrator xprojStick;
    // //OOF
    // RateLimiter xprojRL = new RateLimiter(Robot.dT, this::get_gripperX_cmd, // inputFunc gripperX_cmd
    //         this::measProjection, // phy position func
    //         Robot.arm.MIN_PROJECTION, // output min
    //         Robot.arm.MAX_PROJECTION, // output max
    //         -30.0, // inches/sec // falling rate limit 
    //         30.0, // inches/sec //raising rate limit
    //         InputModel.Position);

    // RateLimiter heightRL = new RateLimiter(Robot.dT, this::get_gripperH_cmd, // gripperH_cmd var as set by this module
    //         this::measHeight, // phy position func
    //         kHeightMin, // output min
    //         kHeightMax, // output max
    //         -40.0, // inches/sec // falling rate limit
    //         40.0, // inches/sec //raising rate limit
    //         InputModel.Position);

    // ExpoShaper xprojShaper = new ExpoShaper(0.5, Robot.m_oi::extensionInput); // joystick defined in m_oi.
    // LimitedIntegrator xprojStick = new LimitedIntegrator(Robot.dT, xprojShaper::get, // shaped joystick input
    //         -20.0, // kGain, 5 in/sec on the joystick (neg. gain, forward stick is neg.)
    //         -20.0, // xmin inches
    //         20.0, // x_max inches
    //         -20.0, // dx_falling rate inch/sec
    //          20.0); // dx_raise rate inch/sec
    // xprojStick.setDeadZone(0.1); // in/sec deadzone

    private double stateH;
    private double stateX;

    public MoveAtHeightDefault() {
        requires(Robot.arm);
        arm = Robot.arm;
        ExpoShaper xprojShaper = new ExpoShaper(0.5, Robot.m_oi::extensionInput); // joystick defined in m_oi.
        xprojStick = new LimitedIntegrator(Robot.dT, xprojShaper::get, // shaped joystick input
                -25.0, // kGain, 5 in/sec on the joystick (neg. gain, forward stick is neg.)
                -25.0, // xmin inches    true pos limit enforced by arm sub-sys
                 25.0, // x_max inches
                -25.0, // dx_falling rate inch/sec
                 25.0); // dx_raise rate inch/sec
        xprojStick.setDeadZone(0.5); // in/sec deadzone
    }

    private double getHeightCommanded() {
        double h_driverOffset = kCapHeight * Robot.m_oi.adjustHeight(); // driver contrib from triggers
        double h = stateH - h_driverOffset; // state machine + driver so both are rate filtered
        return h;
    }

    private double getProjectionCommanded() {
        double x_driverOffset = xprojStick.get(); // co-driver's offset.
        double x = stateX + x_driverOffset;
        return x;
    }

    /**
     * This only works for front side of robot...
     */

    protected void execute() {
        Modes curMode = Robot.m_cmdMgr.getCurMode();
        // read inputs
        double h_cmd = getHeightCommanded();
        double x_cmd = getProjectionCommanded();

        // If target is below pivot height, used for quadrant calcs
        boolean belowPiv = h_cmd < pivotHeight;
        h = Math.abs(pivotHeight - h_cmd);           //h (inches) above or below piviot in mag
        
        //Roughly limit the extension based on game limits and robot geometry
        xProjection = MathUtil.limit(x_cmd, Robot.arm.MIN_PROJECTION, Robot.arm.MAX_PROJECTION);

        double tanRatio = (belowPiv) ? h / xProjection : xProjection / h;
        // Rotate to maintain height as projection changes
        double angle = Math.toDegrees(Math.atan(tanRatio));
        angle += (belowPiv) ? 90.0 : 0.0;
        angle = MathUtil.limit(angle, Robot.arm.PHI_MIN, Robot.arm.PHI_MAX); 
        
        double projLen= Math.sqrt( h*h + xProjection * xProjection);    //total length of arm, from pivot point
        double ext  = projLen - (Robot.arm.ARM_BASE_LENGTH + Robot.arm.WRIST_LENGTH);   // extension required
        

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

    protected void alternativeExecute() {
        Modes curMode = Robot.m_cmdMgr.getCurMode();

        // Get input
        double h_cmd = getHeightCommanded();
        double x_cmd = getProjectionCommanded();

        //Calculate the angle
        double heightAbovePivot = h_cmd - arm.ARM_PIVOT_HEIGHT;
        double calculatedAngle = -Math.toDegrees(Math.atan(heightAbovePivot / x_cmd)) + 90;
        double curAngle = MathUtil.limit(calculatedAngle, arm.PHI_MIN, arm.PHI_MAX);
        double calculatedExtension = (x_cmd / Math.cos(Math.toRadians(90 - arm.getAbsoluteAngle()))) - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH;
        double extensionLength = MathUtil.limit(calculatedExtension, arm.EXTEND_MIN, arm.EXTEND_MAX);

        arm.setAngle(curAngle);
        arm.setExtension(extensionLength);
    }

    protected boolean isFinished() {
        return false;
    }
}