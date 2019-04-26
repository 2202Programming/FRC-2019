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
    public static final double heightAdjustCap = 4.0; // inch/joy units TODO: put in better place
    public static final double kHeightMin = 2.0; // inches
    public static final double kHeightMax = 96.0; // TODO: Find real max

    // Positions in form (InversionState, Height, Position)
    public static final double DeliveryCargoPositions[][][] = { { { 26.875, 23 }, { 54.0, 17.5 }, { 84.0, 17.5 } },
            { { 28.875, -23 }, { 57.0, -17.5 }, { 86.0, -17.5 } } };
    public static final double DeliveryHatchPositions[][][] = { { { 26.5, 23 }, { 55.0, 17.5 }, { 82.0, 17.5 } },
            { { 30.35, -23 }, { 57.375, -17.5 }, { 83.75, -17.5 } } };
    public static final double HuntPositions[][][] = { { { 5.5, 23.0 }, { 18, 22 }, { 24.75, 24.0 } },
            { { 7.0, -23.0 }, { 19.5, -23.5 }, { 28.0, -24.0 } } }; // 0: Floor, 1: Cargo, 2: Hatch
    public static final double[][][] DrivePositions = { { { 49.5, 14.0 }, { 50, 12.0 } },
            { { 51.5, -14.0 }, { 52, -12.0 } } };

    private double stateH;
    private double stateP;
    private Modes prevMode;
    private int prevIndex;
    private boolean checkInverted;

    private RateLimiter heightLimiter;
    private RateLimiter projectionLimiter;
    private LimitedIntegrator projectionAdjustLimiter;

    public ArmStatePositioner() {
        requires(Robot.arm);
        arm = Robot.arm;

        ExpoShaper projectionShaper = new ExpoShaper(0.5, Robot.m_oi::extensionInput); // joystick defined in m_oi.
        projectionAdjustLimiter = new LimitedIntegrator(Robot.dT, projectionShaper::get, // shaped joystick input
                -30.0, // kGain, 25 in/sec on the joystick (neg. gain, forward stick is neg.)
                -25.0, // xmin inches true pos limit enforced by arm sub-sys
                25.0, // x_max inches
                -30.0, // dx_falling rate inch/sec
                30.0); // dx_raise rate inch/sec
        projectionAdjustLimiter.setDeadZone(0.5); // in/sec deadzone

        projectionLimiter = new RateLimiter(Robot.dT, this::getProjectionCommanded, // inputFunc gripperX_cmd
                arm::getProjection, // phy position func
                Robot.arm.MIN_PROJECTION, // output min
                Robot.arm.MAX_PROJECTION, // output max
                -75.0, // inches/sec // falling rate limit
                75.0, // inches/sec //raising rate limit
                InputModel.Position);

        heightLimiter = new RateLimiter(Robot.dT, this::getHeightCommanded, // gripperH_cmd var as set by this module
                arm::getHeight, // phy position func
                kHeightMin, // output min
                kHeightMax, // output max
                -100.0, // inches/sec // falling rate limit
                100.0, // inches/sec //raising rate limit
                InputModel.Position);
        checkInverted = arm.isInverted();
    }

    @Override
    protected void initialize() {
        heightLimiter.initialize();
        projectionLimiter.initialize();
        projectionAdjustLimiter.initialize();
    }

    @Override
    protected void execute() {
        // Update position based on current mode
        Modes curMode = Robot.m_cmdMgr.getCurMode();
        int positionIndex = Robot.m_cmdMgr.getPositionIndex();
        if (curMode != prevMode || positionIndex != prevIndex) {
            // Update position only if state changes to allow something to override position
            // for that state
            System.out.println("Switch States from " + prevMode + " to " + curMode);
            // Update Ratelimiter if we just flipped
            if (checkInverted != arm.isInverted()) {
                initialize();
                checkInverted = arm.isInverted();
                return;
            }
            updatePosition(curMode, positionIndex);
        }

        if (curMode.equals(Modes.Construction) || curMode.equals(Modes.SettingZeros)) {
            return;
        }

        // Get input
        heightLimiter.execute();
        projectionLimiter.execute();
        projectionAdjustLimiter.execute();
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
        if (Math.abs(heightAbovePivot) >= 1e-6) {
            calculatedAngle = (90 - Math.toDegrees(Math.atan(heightAbovePivot / Math.abs(x_cmd)))) * Math.signum(x_cmd);
        }
        double curAngle = MathUtil.limit(calculatedAngle, arm.PHI_MIN, arm.PHI_MAX);

        // Calculate extension based on current angle
        double calculatedExtension = arm.STARTING_EXTENSION;
        if (Math.abs(curAngle) >= 1e-6) {
            calculatedExtension = (x_cmd / Math.sin(Math.toRadians(curAngle))) - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH;
        }
        // setExtension
        double extensionLength = MathUtil.limit(calculatedExtension, arm.EXTEND_MIN, arm.EXTEND_MAX);

        arm.setAngle(curAngle);
        arm.setExtension(extensionLength);
    }

    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void interrupted() {
        Modes curMode = Robot.m_cmdMgr.getCurMode();
        int positionIndex = Robot.m_cmdMgr.getPositionIndex();
        if (curMode != prevMode || positionIndex != prevIndex) {
            // Update position only if state changes to allow something to override position
            // for that state
            updatePosition(curMode, positionIndex);
        }
    }

    private void updatePosition(Modes curMode, int index) {
        int invert = arm.isInverted() ? 1 : 0;
        boolean resetProjectionAdjust = true;
        switch (curMode) {
        case Construction:
            break;
        case SettingZeros:
            stateH = arm.getHeight();
            stateP = arm.getProjection();
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
            resetProjectionAdjust = false;
            break;
        default:
            break;
        }
        prevMode = curMode;
        prevIndex = index;
        if(resetProjectionAdjust) {
            projectionAdjustLimiter.setX(0.0);
        }
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
        int invertMultiplier = Robot.arm.isInverted()? -1 : 1;
        double x_driverOffset = invertMultiplier * projectionAdjustLimiter.get(); // co-driver's offset.
        double x = stateP + x_driverOffset;
        return x;
    }

    /**
     * Set's the arm's position
     * Only used by external commands
     * @param height
     * @param projection
     */
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

    public RateLimiter getHeightLimiter() {
        return heightLimiter;
    }

    public RateLimiter getProjectionLimiter() {
        return projectionLimiter;
    }

    public LimitedIntegrator getDriverAdjustLimiter() {
        return projectionAdjustLimiter;
    }
}