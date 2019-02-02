package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
/**
 * Opens the cargo trap on the robot chassis.
 */
public class OpenCargoTrapCommand extends InstantCommand {
    /**
     * Constructor for the open cargo trap command. Requires a cargoTrap subsystem.
     */
    public OpenCargoTrapCommand() {
        requires(Robot.cargoTrap);
    }
    /**
     * Traps the cargo within the arms.
     */
    @Override
    protected void execute() {
        Robot.cargoTrap.openTrapArms();
    }
}