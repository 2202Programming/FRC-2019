package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
/**
 * Retracts the cargo trap on the robot chassis.
 */
public class RetractCargoTrapCommand extends InstantCommand {
    /**
     * Constructor for the retract cargo trap command. Requires a cargoTrap subsystem.
     */
    public RetractCargoTrapCommand() {
        addRequirements(Robot.cargoTrap);
    }
    /**
     * Retracts the cargo trap.
     */
    @Override
    public void execute() {
        Robot.cargoTrap.retractTrap();
        Robot.cargoTrap.setIntake(0.0);
    }
}