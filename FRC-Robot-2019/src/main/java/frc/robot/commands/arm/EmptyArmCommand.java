package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.TimedCommand;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class EmptyArmCommand extends TimedCommand {
    ArmSubsystem arm = Robot.arm;

    public EmptyArmCommand(double time) {
        super(time);
        addRequirements(arm);
    }

}
