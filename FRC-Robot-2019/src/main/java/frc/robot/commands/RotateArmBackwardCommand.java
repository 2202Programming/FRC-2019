import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;

public class RotateArmBackwardCommand extends Command {
    public RotateArmBackwardCommand() {
        requires(Robot.arm);
    }

    @Override
    protected void initialize() {
        Robot.arm.stopRotation();
    }

    @Override
    protected void execute() {
        Robot.arm.rotateBackward();
    }

    @Override
    protected void end() {
        Robot.arm.stopRotation();
    }

    @Override
    protected boolean isFinished() {
        return Robot.m_oi.getController1().getRawButtonReleased(XboxControllerButtonCode.B.getCode()) || Robot.arm.rotationAtMin();
    }
}