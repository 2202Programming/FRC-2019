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

public class MoveAtHeightDefault extends Command {
    private ArmSubsystem arm;
    // Height of point of rotation for the arm in inches
    public final double heightAdjustCap = 4.0; // inch/joy units TODO: put in better place
    public final double kHeightMin = 2.0; // inches
    public final double kHeightMax = 96.0; // inches

    // Make an h' to more easily construct a triangle, relative to pivot height
    private double h;

    // Projection of the arm on the ground
    private double xProjection;

    final double DeliveryCargoHeights[] = { 26.875, 55.0, 84.0 }; // TODO: fix the numbers
    final double DeliveryHatchHeights[] = { 27.5, 55.0, 82.0 };
    final double deliveryProjection[] = { 25.0, 25.0, 25.0 }; // TODO: fix the numbers

    private RateLimiter heightLimiter;
    private RateLimiter projectionLimiter;
    private LimitedIntegrator projectionAdjustLimiter;

    private double stateH;
    private double stateX;

    public MoveAtHeightDefault() {
        requires(Robot.arm);
        arm = Robot.arm;

        ExpoShaper projectionShaper = new ExpoShaper(0.5, Robot.m_oi::extensionInput); // joystick defined in m_oi.
        projectionAdjustLimiter = new LimitedIntegrator(Robot.dT, projectionShaper::get, // shaped joystick input
                -25.0, // kGain, 20 in/sec on the joystick (neg. gain, forward stick is neg.)
                -25.0, // xmin inches true pos limit enforced by arm sub-sys
                25.0, // x_max inches
                -25.0, // dx_falling rate inch/sec
                25.0); // dx_raise rate inch/sec
        projectionAdjustLimiter.setDeadZone(0.5); // in/sec deadzone

        projectionLimiter = new RateLimiter(Robot.dT, this::getProjectionCommanded, // inputFunc gripperX_cmd
                arm::getProjection, // phy position func
                Robot.arm.MIN_PROJECTION, // output min
                Robot.arm.MAX_PROJECTION, // output max
                -50.0, // inches/sec // falling rate limit
                50.0, // inches/sec //raising rate limit
                InputModel.Position);

        heightLimiter = new RateLimiter(Robot.dT, this::getHeightCommanded, // gripperH_cmd var as set by this module
                arm::getHeight, // phy position func
                kHeightMin, // output min
                kHeightMax, // output max
                -80.0, // inches/sec // falling rate limit
                80.0, // inches/sec //raising rate limit
                InputModel.Position);
    }

    private double getHeightCommanded() {
        double h_driverOffset = heightAdjustCap * Robot.m_oi.adjustHeight(); // driver contrib from triggers
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
        heightLimiter.execute();
        projectionLimiter.execute();
        double h_cmd = heightLimiter.get();
        double x_cmd = projectionLimiter.get();
        double heightAbovePivot = h_cmd - arm.ARM_PIVOT_HEIGHT;

        // Adjusts x_cmd so h_cmd is always reached
        if (heightAbovePivot * heightAbovePivot + x_cmd * x_cmd >= arm.MAX_ARM_LENGTH * arm.MAX_ARM_LENGTH) {
            // Assumes h_cmd doesn't go above the max attainable height
            x_cmd = Math.sqrt(arm.MAX_ARM_LENGTH * arm.MAX_ARM_LENGTH - heightAbovePivot * heightAbovePivot);
        }

        // Calculate the angle
        double calculatedAngle = 0.0;
        if (Math.abs(x_cmd) <= 1e-6) {
            calculatedAngle = 90 - Math.toDegrees(Math.atan(heightAbovePivot / x_cmd));
        }
        double curAngle = MathUtil.limit(calculatedAngle, arm.PHI_MIN, arm.PHI_MAX);

        // Calculate extension based on current angle
        double calculatedExtension = heightAbovePivot;
        if (Math.abs(arm.getRealAngle()) <= 1e-6) {
            calculatedExtension = (x_cmd / Math.sin(arm.getRealAngle())) - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH;
        }
        // Limiting here is technically unnecessary because limiting is also done in
        // setExtension
        double extensionLength = MathUtil.limit(calculatedExtension, arm.EXTEND_MIN, arm.EXTEND_MAX);

        arm.setAngle(curAngle);
        arm.setExtension(extensionLength);
    }

    protected boolean isFinished() {
        return false;
    }
}