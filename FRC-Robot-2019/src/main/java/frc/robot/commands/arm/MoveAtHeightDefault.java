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
 * Billy & X think this is working for forward facing arm.
 * 
 * 2/20/19 DPL adjusted to take h and l commands from functions cleaned up
 * constraints
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

    private LimitedIntegrator projectionAdjustLimiter;

    private double stateH;
    private double stateX;

    public MoveAtHeightDefault(RateLimiter projectionLimiter, RateLimiter heightLimiter, LimitedIntegrator projectionAdjustLimiter) {
        requires(Robot.arm);
        arm = Robot.arm;
        
        this.projectionAdjustLimiter = projectionAdjustLimiter;
    }

    private double getHeightCommanded() {
        double h_driverOffset = kCapHeight * Robot.m_oi.adjustHeight(); // driver contrib from triggers
        double h = stateH - h_driverOffset; // state machine + driver so both are rate filtered
        return h;
    }

    private double getProjectionCommanded() {
        double x_driverOffset = projectionAdjustLimiter.get(); // co-driver's offset.
        double x = stateX + x_driverOffset;
        return x;
    }

    @Override
    protected void execute() {
        Modes curMode = Robot.m_cmdMgr.getCurMode();

        // Get input
        double h_cmd = getHeightCommanded();
        double x_cmd = getProjectionCommanded();

        // Calculate the angle
        double heightAbovePivot = h_cmd - arm.ARM_PIVOT_HEIGHT;
        double calculatedAngle = -Math.toDegrees(Math.atan(heightAbovePivot / x_cmd)) + 90;
        double curAngle = MathUtil.limit(calculatedAngle, arm.PHI_MIN, arm.PHI_MAX);
        double calculatedExtension = (x_cmd / Math.cos(Math.toRadians(90 - arm.getAbsoluteAngle())))
                - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH;
        //Limiting here is technically unnecessary because limiting is also done in setExtension
        double extensionLength = MathUtil.limit(calculatedExtension, arm.EXTEND_MIN, arm.EXTEND_MAX); 

        arm.setAngle(curAngle);
        arm.setExtension(extensionLength);
    }

    protected boolean isFinished() {
        return false;
    }
}