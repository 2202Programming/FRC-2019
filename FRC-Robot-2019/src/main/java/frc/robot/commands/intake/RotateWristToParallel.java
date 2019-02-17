package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class RotateWristToParallel extends InstantCommand {
    private double targetAngle;

    public RotateWristToParallel() {
        requires(Robot.intake);
        targetAngle = Robot.arm.getAngle() - 90;
    }

    @Override
    protected void execute() {
        Robot.intake.setAngle(targetAngle);
    }
}