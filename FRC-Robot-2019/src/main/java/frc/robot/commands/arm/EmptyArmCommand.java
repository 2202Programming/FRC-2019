package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class EmptyArmCommand extends WaitCommand {
    ArmSubsystem arm = Robot.arm;

    public EmptyArmCommand(double time) {
        super(time);
        addRequirements(Robot.arm);
    }

}
