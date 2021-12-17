package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Robot;

public class ExtendArmToPositionCommand extends CommandBase {
    final private double kTolerance = 0.50;  //(inches)
    private double distance;

    public ExtendArmToPositionCommand(double distance) {
        this.distance = distance;
        addRequirements((Subsystem)Robot.arm);
    }

    @Override
    public void execute() {
        Robot.arm.setExtension(distance);
    }

    public boolean isFinished() {
        return Math.abs(Robot.arm.getExtension() - distance) < kTolerance;   
    }
}