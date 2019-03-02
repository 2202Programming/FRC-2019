package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class ClimbMotorTestCmd extends InstantCommand {
    // On state
    double speed;

    public ClimbMotorTestCmd(double speed) {
        this.setName("Climb Motor=" + speed);
        this.speed = speed;
    }

    @Override
    protected void initialize() {
        Robot.climber.setExtenderSpeed(speed);
    }

    @Override
    protected void execute() {
        Robot.climber.setExtenderSpeed(speed);
    }

    @Override
    protected void end() {
        Robot.climber.setExtenderSpeed(0);        
    }
}
