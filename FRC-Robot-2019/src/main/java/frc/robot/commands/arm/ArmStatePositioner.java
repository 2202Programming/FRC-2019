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

public class ArmStatePositioner extends Command {
    private ArmSubsystem arm;
    // Height of point of rotation for the arm in inches
    public final double heightAdjustCap = 4.0; // inch/joy units TODO: put in better place
    public final double kHeightMin = 2.0; // inches
    public final double kHeightMax = 96.0; // TODO: Find real max

    // Positions in form (InversionState, Height, Position)
    public static final double DeliveryCargoPositions[][][] = { { { 26.875, 25.0 }, { 55.0, 25.0 }, { 84.0, 25.0 } },
            { { 26.875, -25.0 }, { 55.0, -25.0 }, { 84.0, -25.0 } } };
    public static final double DeliveryHatchPositions[][][] = { { { 27.5, 25.0 }, { 55.0, 25.0 }, { 82.0, 25.0 } },
            { { 27.5, -25.0 }, { 55.0, -25.0 }, { 82.0, -25.0 } } };
    public static final double HuntPositions[][][] = { { { 5.0, 23.0 }, { 17.5, 23.5 }, { 24.0, 24.0 } },
            { { 5.0, -23.0 }, { 17.5, -23.5 }, { 24.0, -24.0 } } }; // 0: Floor, 1: Cargo, 2: Hatch
    public static final double[][][] DrivePositions = { { { 49.5, 14.0 }, { 50, 12.0 } }, { { 49.5, -14.0 }, { 50, -12.0 } } };

    private double stateH;
    private double stateP;
    private Modes prevMode;

    private RateLimiter heightLimiter;
    private RateLimiter projectionLimiter;
    private LimitedIntegrator projectionAdjustLimiter;

    public ArmStatePositioner() {
        requires(Robot.arm);
        arm = Robot.arm;

        ExpoShaper projectionShaper = new ExpoShaper(0.5, Robot.m_oi::extensionInput); // joystick defined in m_oi.
        projectionAdjustLimiter = new LimitedIntegrator(Robot.dT, projectionShaper::get, // shaped joystick input
                -25.0, // kGain, 25 in/sec on the joystick (neg. gain, forward stick is neg.)
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

    @Override
    protected void initialize() {
        heightLimiter.initialize();
        projectionLimiter.initialize();
        projectionAdjustLimiter.initialize();
        stateH = arm.getHeight();
        stateP = arm.getProjection();
    }

    @Override
    protected void execute() {
        // Update position based on current mode
        Modes curMode = Robot.m_cmdMgr.getCurMode();
        int positionIndex = Robot.m_cmdMgr.getPositionIndex();
        if(curMode != prevMode) {
            // Update position only if state changes to allow something to override position for that state
            updatePosition(curMode, positionIndex);
        }

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
        if (Math.abs(x_cmd) >= 1e-6) {
            calculatedAngle = 90 - Math.toDegrees(Math.atan(heightAbovePivot / x_cmd));
        }
        double curAngle = MathUtil.limit(calculatedAngle, arm.PHI_MIN, arm.PHI_MAX);

        // Calculate extension based on current angle
        double calculatedExtension = heightAbovePivot;
        if (Math.abs(arm.getRealAngle()) >= 1e-6) {
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

    private void updatePosition(Modes curMode, int index) {
        int invert = getInversionStatus() ? 1 : 0;
        switch (curMode) {
        case Construction:
            break;
        case SettingZeros:
            break;
        case HuntGameStart:
            break;
        case HuntingHatch:
            stateH = HuntPositions[invert][2][0];
            stateP = HuntPositions[invert][2][1];
            break;
        case HuntingCargo:
            stateH = HuntPositions[invert][1][0];
            stateP = HuntPositions[invert][1][1];
            break;
        case HuntingFloor:
            stateH = HuntPositions[invert][0][0];
            stateP = HuntPositions[invert][0][1];
            break;
        case Drive:
            stateH = DrivePositions[invert][index][0];
            stateP = DrivePositions[invert][index][1];
            break;
        case DeliverHatch:
            stateH = DeliveryHatchPositions[invert][index][0];
            stateP = DeliveryHatchPositions[invert][index][1];
            break;
        case DeliverCargo:
            stateH = DeliveryCargoPositions[invert][index][0];
            stateP = DeliveryCargoPositions[invert][index][1];
            break;
        case Flipping:
            break;
        case Releasing:
            break;
        default:
            break;
        }
        prevMode = curMode;
    }

    /**
     * Gets whether the arm is inverted
     * 
     * @return Inversion status
     */
    public boolean getInversionStatus() {
        return arm.getRealAngle() < 0;
    }

    public double getStateHeight() {
        return stateH;
    }

    public double getStateProjection() {
        return stateP;
    }
 
    public double getHeightCommanded() {
        double h_driverOffset = heightAdjustCap * Robot.m_oi.adjustHeight(); // driver contrib from triggers
        double h = stateH - h_driverOffset; // state machine + driver so both are rate filtered
        return h;
    }

    public double getProjectionCommanded() {
        double x_driverOffset = projectionAdjustLimiter.get(); // co-driver's offset.
        double x = stateP + x_driverOffset;
        return x;
    }

    public void setPosition(double height, double projection) {
        stateH = height;
        stateP = projection;
    }

    public void setHeightLimiter(double minHeight, double maxHeight, double fallSpeed, double raiseSpeed) {
        heightLimiter.setConstraints(minHeight, maxHeight, fallSpeed, raiseSpeed);
    }

    public void setProjectionLimiter(double minProjection, double maxProjection, double retractSpeed,
            double extendSpeed) {
        projectionLimiter.setConstraints(minProjection, maxProjection, retractSpeed, extendSpeed);
    }

    public void setDriverAdjustLimiter(double inputGain, double minExtension, double maxExtension, double retractSpeed,
            double extendSpeed) {
        projectionAdjustLimiter.setConstraints(inputGain, minExtension, maxExtension, retractSpeed, extendSpeed);
    }
}