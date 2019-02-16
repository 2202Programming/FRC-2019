package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TestRotateArmToAngleCommand extends Command {
    final private double kTolerance = 2.0;
    private double angle;

    public TestRotateArmToAngleCommand(double angle) {
        this.angle = angle;
        requires(Robot.arm);
    }

    @Override
    protected void execute() {
    }

    protected boolean isFinished() {
        return Math.abs(Robot.arm.getAngle() - angle) < kTolerance;   
    }

}