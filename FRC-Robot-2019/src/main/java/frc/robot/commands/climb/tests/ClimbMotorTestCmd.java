package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class ClimbMotorTestCmd extends Command {
    // On state
    double speed;

    public ClimbMotorTestCmd(double speed) {
        this.setName("Climb Motor=" + speed);
        this.speed = speed;
    }

    @Override
    protected void initialize() {
    }

    /**
     * extend or retract until a limit is hit
     * +speed is extend, -speed is retract
     */
    @Override
    protected void execute() {
        double s = ((Robot.climber.footAtExtend()  && speed > 0.0 ) ||
                    (Robot.climber.footAtRetract() && speed < 0.0)) ? 0.0 : speed;
        Robot.climber.setExtenderSpeed(s);
    }

    @Override
    protected void end() {
        Robot.climber.setExtenderSpeed(0);        
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
