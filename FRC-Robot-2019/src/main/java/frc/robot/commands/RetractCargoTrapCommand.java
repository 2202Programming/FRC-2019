package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class RetractCargoTrapCommand extends InstantCommand {
    public RetractCargoTrapCommand() {
        requires(Robot.cargoTrap);
    }

    @Override
    protected void execute() {
        Robot.cargoTrap.retractTrap();
    }
}