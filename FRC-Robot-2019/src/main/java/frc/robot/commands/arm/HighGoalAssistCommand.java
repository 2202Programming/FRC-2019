package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.ExtendArmToPositionCommand;
import frc.robot.commands.arm.RotateArmToAngleCommand;

public class HighGoalAssistCommand extends CommandGroup {
    public HighGoalAssistCommand() {
        addSequential(new RotateArmToAngleCommand(60));
        addSequential(new ExtendArmToPositionCommand(12));
    }
}