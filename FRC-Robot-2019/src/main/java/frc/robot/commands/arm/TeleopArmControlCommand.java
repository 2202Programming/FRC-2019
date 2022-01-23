package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.util.MathUtil;

public class TeleopArmControlCommand extends CommandBase {
    private ArmSubsystem arm;
    DoubleSupplier heightCmdFunct;
    DoubleSupplier projectionCmdFunct;

    private double height_cmd;
    private double projection_cmd;
    XboxController in;
    // private PositionEnum[] orderedPositions = { PositionEnum.CargoLow, PositionEnum.CargoMid, PositionEnum.CargoHigh,
    //        PositionEnum.HatchLow, PositionEnum.HatchMid, PositionEnum.HatchHigh };

    public TeleopArmControlCommand(DoubleSupplier heightCmdFunct, DoubleSupplier projectionCmdFunct) {
        addRequirements(Robot.arm);
        arm = Robot.arm;
        this.heightCmdFunct = heightCmdFunct;
        this.projectionCmdFunct = projectionCmdFunct;

        //TODO: fix the way button is handled. 
        in = Robot.m_oi.getAssistantController();
    }

    @Override
   public void initialize() {
        //dpl - can't reset encoders when command start.  Command could get enabled multiple times
        // read initial positions
        readInputs();
    }

    private void readInputs() {
        height_cmd = heightCmdFunct.getAsDouble();
        projection_cmd = projectionCmdFunct.getAsDouble();
    }

    @Override
    public void execute() {
        updatePositionVector();
        double heightAbovePivot = height_cmd - arm.ARM_PIVOT_HEIGHT;
        double curAngle = -Math.toDegrees(Math.atan(heightAbovePivot / projection_cmd)) + 90;
        //double extensionLength = limit(0, arm.EXTEND_MAX, Math.sqrt(heightAbovePivot * heightAbovePivot + projection_cmd * projection_cmd) - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH);
        double calculatedExtension = (projection_cmd / Math.cos(Math.toRadians(90 - arm.getRealAngle()))) - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH;
        double extensionLength = MathUtil.limit(calculatedExtension, arm.EXTEND_MIN, arm.EXTEND_MAX);
        
        SmartDashboard.putNumber("Current Height : ", height_cmd);
        SmartDashboard.putNumber("Current Projection: ", projection_cmd);
        SmartDashboard.putNumber("Arm Angle: ", curAngle);
        SmartDashboard.putNumber("Extension Length: ", extensionLength);


        arm.setAngle(curAngle);
        arm.setExtension(extensionLength);
    }

    //TODO: DEREK/BILLY compare states in the controlManager for heights
    private void updatePositionVector() {
        //TODO Implement states
        if (in.getLeftBumper()) {
            // Go to Lower State
        } else if (in.getRightBumper()) {
            // Go To Higher State
        } else {
            // TODO: Bind to real controls and add rate limiting
            double changeInHeight = heightCmdFunct.getAsDouble();
            double changeInProjection = projectionCmdFunct.getAsDouble();;

            // TODO: Limit these values so they don't break physical constraints
            height_cmd = MathUtil.limit(height_cmd + changeInHeight, 12, 70);
            projection_cmd = MathUtil.limit(projection_cmd + changeInProjection, arm.MIN_PROJECTION, arm.MAX_PROJECTION);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}