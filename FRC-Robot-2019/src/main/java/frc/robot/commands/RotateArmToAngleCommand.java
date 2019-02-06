package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RotateArmToAngleCommand extends Command {
    private double position;

    public RotateArmToAngleCommand(double position) {
        this.position = position;
        requires(Robot.arm);
    }

    @Override
    protected void execute() {
        Robot.arm.rotateToPosition(position);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}