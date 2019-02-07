package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WaitForCargoCommand extends Command {
    public WaitForCargoCommand() {
        requires(Robot.cargoTrap);
    }

    @Override
    protected boolean isFinished() {
       // return Robot.cargoTrap.containsCargo();
       return true;
    }
}