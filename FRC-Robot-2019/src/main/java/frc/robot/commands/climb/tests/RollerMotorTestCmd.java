package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class RollerMotorTestCmd extends Command {
    // On state
    double speed;

    public RollerMotorTestCmd(double speed) {
        this.setName("Climb Motor=" + speed);
        this.speed = speed;
    }

    @Override
    protected void initialize() {
        Robot.climber.setRollerSpeed(speed);
        System.out.println("Initialize Roller");
    }

    @Override
    protected void execute() {
        Robot.climber.setRollerSpeed(speed);
        System.out.println("Initialize Execute");
    }

    @Override
    protected void end() {
        Robot.climber.setRollerSpeed(0);        
        System.out.println("Initialize End");
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
