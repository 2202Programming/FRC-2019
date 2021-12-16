package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ExtendArmToPositionCommand extends CommandBase {
    final private double kTolerance = 0.50;  //(inches)
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