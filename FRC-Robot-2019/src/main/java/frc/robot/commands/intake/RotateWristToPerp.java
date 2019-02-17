package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class RotateWristToPerp extends InstantCommand {

    public RotateWristToPerp() {
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        double phi = Robot.arm.getAngle();
        double target = phi - 180;
        Robot.intake.setAngle(target);
    }
}