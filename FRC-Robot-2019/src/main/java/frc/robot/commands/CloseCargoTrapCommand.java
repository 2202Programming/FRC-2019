package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class CloseCargoTrapCommand extends InstantCommand {
    public CloseCargoTrapCommand() {
        requires(Robot.cargoTrap);
    }

    @Override
    protected void execute() {
        Robot.cargoTrap.closeTrapArms();
    }
}