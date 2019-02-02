import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;

public class RetractArmCommand extends Command {
    public RetractArmCommand() {
        requires(Robot.arm);
    }

    @Override
    protected void initialize() {
        Robot.arm.stopExtension();
    }

    @Override
    protected void execute() {
        Robot.arm.retract();
    }

    @Override
    protected void end() {
        Robot.arm.stopExtension();
    }

    @Override
    protected boolean isFinished() {
        return Robot.m_oi.getController1().getRawButtonReleased(XboxControllerButtonCode.A.getCode()) || Robot.arm.extensionAtMin();
    }
}