package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class TeleopArmControlCommand extends Command {
    private ArmSubsystem arm;
    private XboxController in;
    private double height_cmd;
    private double projection_cmd;

    public TeleopArmControlCommand() {
        requires(Robot.arm);
        arm = Robot.arm;
        in = Robot.m_oi.getAssistantController();
        projection_cmd = 0;
        height_cmd = 0;
    }

    @Override
    protected void initialize() {
        // TODO: Find the real intial values
        arm.zeroArm();
        projection_cmd = 10.5;
        height_cmd = 6.5;
    }

    @Override
    protected void execute() {
        updatePositionVector();
        double heightAbovePivot = height_cmd - arm.ARM_PIVOT_HEIGHT;
        double curAngle = -Math.toDegrees(Math.atan(heightAbovePivot / projection_cmd)) + 90;
        //double extensionLength = limit(0, arm.EXTEND_MAX, Math.sqrt(heightAbovePivot * heightAbovePivot + projection_cmd * projection_cmd) - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH);
        double extensionLength = limit(0, arm.EXTEND_MAX, (projection_cmd / Math.cos(Math.toRadians(90 - arm.getAngle()))) - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH);
        
        SmartDashboard.putNumber("Commanded Height : ", height_cmd);
        SmartDashboard.putNumber("Commanded Projection: ", projection_cmd);
        SmartDashboard.putNumber("Arm Angle: ", curAngle);
        SmartDashboard.putNumber("Extension Length: ", extensionLength);

        arm.setAngle(curAngle);
        //arm.setExtension(extensionLength);
    }

    private void updatePositionVector() {
        //TODO Implement states
        if (in.getBumper(Hand.kLeft)) {
            // Go to Lower State
        } else if (in.getBumper(Hand.kRight)) {
            // Go To Higher State
        } else {
            // TODO: Bind to real controls and add rate limiting
            double changeInHeight = Math.abs(in.getY(Hand.kLeft)) < 0.05? 0: -in.getY(Hand.kLeft);
            double changeInProjection = Math.abs(in.getY(Hand.kRight)) < 0.05? 0: -in.getY(Hand.kRight);

            // TODO: Limit these values so they don't break physical constraints
            height_cmd = limit(5, 70, height_cmd + changeInHeight);
            projection_cmd = limit(arm.MIN_PROJECTION, arm.MAX_PROJECTION, projection_cmd + changeInProjection);
        }
    }

    private double limit(double min, double max, double value) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void interrupted() {
        return;
    }
}