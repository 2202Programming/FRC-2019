package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class DeployCargoTrapCommand extends InstantCommand {
    public DeployCargoTrapCommand() {
        requires(Robot.cargoTrap);
    }

    @Override
    protected void execute() {
        Robot.cargoTrap.deployTrap();
    }
}