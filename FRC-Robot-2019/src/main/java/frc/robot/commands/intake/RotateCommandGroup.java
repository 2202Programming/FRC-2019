package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class RotateCommandGroup extends Command {
    private double angle;
    private double x;
    private ArmSubsystem arm;

    public RotateCommandGroup(double x, double angle) {
        this.angle = angle;
        this.x = x;
        requires(Robot.arm);
        arm = Robot.arm;
    }

    @Override
    protected void execute() {
        arm.setPosition(arm.getAngle() + angle);
        arm.extendToPosition(x / Math.cos(Math.toRadians(angle)));
    }

    protected boolean isFinished() {
        return Math.abs(Robot.arm.getAngle() - angle) < 1;
    }
}