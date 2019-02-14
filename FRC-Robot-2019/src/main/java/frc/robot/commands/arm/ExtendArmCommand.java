package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;

public class ExtendArmCommand extends Command {
    public ExtendArmCommand() {
        requires(Robot.arm);
    }

    @Override
    protected void initialize() {
        Robot.arm.stopExtension();
    }

    @Override
    protected void execute() {
        Robot.arm.extend();
    }

    @Override
    protected void end() {
        Robot.arm.stopExtension();
    }

    @Override
    protected boolean isFinished() {
        return Robot.m_oi.getAssistantController().getRawButtonReleased(XboxControllerButtonCode.Y.getCode());
    }
}