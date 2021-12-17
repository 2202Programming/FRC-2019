package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class WaitForCargoCommand extends CommandBase {
    public WaitForCargoCommand() {
        addRequirements(Robot.cargoTrap);
    }

    @Override
    public boolean isFinished() {
       return Robot.cargoTrap.cargoInSight();
    }
}