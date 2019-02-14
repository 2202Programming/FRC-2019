package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ExtendArmToPositionCommand;
import frc.robot.commands.arm.RotateArmToAngleCommand;
import frc.robot.subsystems.ArmSubsystem;

public class RotateCommandGroup extends CommandGroup {

    public RotateCommandGroup(double x, double angle) {
        requires(Robot.arm);
        ArmSubsystem arm = Robot.arm;
        addParallel(new RotateArmToAngleCommand(arm.getAngle() + angle));
        addParallel(new ExtendArmToPositionCommand(x / Math.cos(Math.toRadians(angle))));
    }
}