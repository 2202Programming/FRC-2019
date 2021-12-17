package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class CheckSolenoids extends WaitCommand {
    public CheckSolenoids() {
        super(0.1);    // waits 0.1 seconds before calling end()
        addRequirements(Robot.climber);
    }
    
   public void initialize() {
        Robot.climber.setDrawerSlide(true);
        Robot.climber.setPawl(true);
    }

    public void end(boolean interrupted) {
        Robot.climber.setDrawerSlide(false);
        Robot.climber.setPawl(false);
    }
}