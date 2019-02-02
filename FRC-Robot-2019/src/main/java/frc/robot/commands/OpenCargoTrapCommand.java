package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class OpenCargoTrapCommand extends InstantCommand {
    public OpenCargoTrapCommand() {
        requires(Robot.cargoTrap);
    }

    @Override
    protected void execute() {
        Robot.cargoTrap.openTrapArms();
    }
}