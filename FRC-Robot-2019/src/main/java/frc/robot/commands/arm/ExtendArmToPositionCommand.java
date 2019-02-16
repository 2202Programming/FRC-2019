package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ExtendArmToPositionCommand extends Command {
    final private double kTolerance = 2.0;
    private double distance;

    public ExtendArmToPositionCommand(double distance) {
        this.distance = distance;
        requires(Robot.arm);
    }

    @Override
    protected void execute() {
        Robot.arm.setExtension(distance);
    }

    protected boolean isFinished() {
        return Math.abs(Robot.arm.getExtension() - distance) < kTolerance;   
    }
}