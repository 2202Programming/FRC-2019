package frc.robot.commands.cargo.tests;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class OuttakeTestCmd extends CommandBase {
    // On state
    double speed;

    public OuttakeTestCmd(double speed) {
        this.setName("Cargo Motor=" + -Math.abs(speed));
        this.speed = -Math.abs(speed);
    }

    @Override
   public void initialize() {
        Robot.cargoTrap.setIntake(speed);
    }

    @Override
    public void execute() {
        Robot.cargoTrap.setIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.cargoTrap.setIntake(0);        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
