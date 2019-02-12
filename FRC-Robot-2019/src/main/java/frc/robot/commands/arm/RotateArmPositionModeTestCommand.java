package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.input.XboxControllerButtonCode;

public class RotateArmPositionModeTestCommand extends Command {
    public RotateArmPositionModeTestCommand() {
        requires(Robot.arm);
    }

    @Override
    protected void initialize() {
        Robot.arm.stopRotation();
    }

    @Override
    protected void execute() {
        
    }

    @Override
    protected void end() {
        Robot.arm.stopRotation();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}