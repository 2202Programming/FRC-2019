package frc.robot.commands.cargo.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class OuttakeTestCmd extends Command {
    // On state
    double speed;

    public OuttakeTestCmd(double speed) {
        this.setName("Cargo Motor=" + -Math.abs(speed));
        this.speed = -Math.abs(speed);
    }

    @Override
    protected void initialize() {
        Robot.cargoTrap.setIntake(speed);
    }

    @Override
    protected void execute() {
        Robot.cargoTrap.setIntake(speed);
    }

    @Override
    protected void end() {
        Robot.cargoTrap.setIntake(0);        
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
