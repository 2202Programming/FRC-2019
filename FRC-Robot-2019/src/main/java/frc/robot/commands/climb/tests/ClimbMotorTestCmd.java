package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class ClimbMotorTestCmd extends CommandBase {
    // On state
    double speed;

    public ClimbMotorTestCmd(double speed) {
        this.setName("Climb Motor=" + speed);
        this.speed = speed;
    }

    @Override
   public void initialize() {
    }

    /**
     * extend or retract until a limit is hit
     * +speed is extend, -speed is retract
     */
    @Override
    public void execute() {
        double s = ((Robot.climber.footAtExtend()  && speed > 0.0 ) ||
                    (Robot.climber.footAtRetract() && speed < 0.0)) ? 0.0 : speed;
        Robot.climber.setExtenderSpeed(s);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.setExtenderSpeed(0);        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
