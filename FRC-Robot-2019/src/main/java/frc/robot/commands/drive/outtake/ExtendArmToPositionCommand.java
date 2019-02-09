package frc.robot.commands.drive.outtake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ExtendArmToPositionCommand extends Command {
    private double position;

    public ExtendArmToPositionCommand(double position) {
        this.position = position;
        requires(Robot.arm);
    }

    @Override
    protected void execute() {
        Robot.arm.extendToPosition(position);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}