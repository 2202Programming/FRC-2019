package frc.robot.commands.cargo.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class IntakeTestCmd extends Command {
    // On state
    double speed;

    public IntakeTestCmd(double speed) {
        this.setName("Cargo Motor=" + Math.abs(speed));
        this.speed = Math.abs(speed);
    }

    @Override
    protected void initialize() {
        Robot.climber.setIntake(speed);
    }

    @Override
    protected void execute() {
        Robot.climber.setIntake(speed);
    }

    @Override
    protected void end() {
        Robot.climber.setIntake(0);        
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
