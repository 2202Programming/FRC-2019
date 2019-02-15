package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ExtendArmToPositionCommand;
import frc.robot.commands.arm.RotateArmToAngleCommand;
import frc.robot.subsystems.ArmSubsystem;

public class RotateCommandGroup extends CommandGroup {

    /*
    This command should probably just take in a height or a height and ground projection
    We'll want to maximize ground projection most of the time
    */
    public RotateCommandGroup(double x, double angle) {
        requires(Robot.arm);
        ArmSubsystem arm = Robot.arm;
        addParallel(new RotateArmToAngleCommand(arm.getAngle() + angle));

        /*
        TODO: Convert angle and extension to correct frame of reference
        This math is over simplified on extension
        Doesn't factor in the length of the arm not accounting for the part that doesn't extend (add a constant)
        */
        addParallel(new ExtendArmToPositionCommand(x / Math.cos(Math.toRadians(angle))));
    }
}