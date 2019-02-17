package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class RotateWristToPerp extends InstantCommand {
    private double targetAngle;

    public RotateWristToPerp() {
        requires(Robot.intake);
        targetAngle = Robot.arm.getAngle() - 180;
    }

    @Override
    protected void execute() {
        Robot.intake.setAngle(targetAngle);
    }
}