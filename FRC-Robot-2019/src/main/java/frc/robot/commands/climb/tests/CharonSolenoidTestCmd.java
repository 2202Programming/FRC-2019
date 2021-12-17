package frc.robot.commands.climb.tests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

/**
 * 
 * Instantiate with one or other for what you need.
 */
public class CharonSolenoidTestCmd extends InstantCommand {
    // On state
    boolean enabled;

    public CharonSolenoidTestCmd(boolean enabled) {
        this.setName("Charon=" + enabled);
        this.enabled = enabled;
    }

    @Override
   public void initialize() {
       Robot.climber.setDrawerSlide(enabled); 
    }
  
}
