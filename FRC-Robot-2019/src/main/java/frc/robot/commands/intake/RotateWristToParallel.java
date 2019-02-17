package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class RotateWristToParallel extends InstantCommand {

    public RotateWristToParallel() {
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        double phi = Robot.arm.getAngle();
        double target = phi - 90;
        Robot.intake.setAngle(target);
    }
}