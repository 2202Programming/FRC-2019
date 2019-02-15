package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.PositionEnum;
import frc.robot.subsystems.ArmSubsystem;

public class TeleopArmControlCommand extends Command {
    private ArmSubsystem arm;
    private XboxController in;
    private double curHeight;
    private double curProjection;
    private PositionEnum[] orderedPositions = { PositionEnum.CargoLow, PositionEnum.CargoMid, PositionEnum.CargoHigh,
            PositionEnum.HatchLow, PositionEnum.HatchMid, PositionEnum.HatchHigh };

    public TeleopArmControlCommand() {
        requires(Robot.arm);
        arm = Robot.arm;
        in = Robot.m_oi.getAssistantController();
        curProjection = 0;
        curHeight = 0;
    }

    @Override
    protected void initialize() {
        // TODO: Find the real intial values
        curProjection = 0;
        curHeight = 0;
    }

    @Override
    protected void execute() {
        double armHeight = curHeight - arm.ARM_HEIGHT;
        double curAngle = Math.toDegrees(Math.atan(armHeight / curProjection));
        double extensionLength = Math.sqrt(armHeight * armHeight + curProjection * curProjection);

        arm.setAngle(curAngle);
        arm.extendToPosition(extensionLength);
    }

    private void updatePositionVector() {
        //TODO Implement states
        if (in.getBumper(Hand.kLeft)) {
            // Go to Lower State
        } else if (in.getBumper(Hand.kRight)) {
            // Go To Higher State
        } else {
            // TODO: Bind to real controls and add rate limiting
            double changeInHeight = in.getY(Hand.kLeft);
            double changeInProjection = in.getY(Hand.kRight);

            // TODO: Limit these values so they don't break physical constraints
            curHeight += changeInHeight;
            curProjection += changeInProjection;
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