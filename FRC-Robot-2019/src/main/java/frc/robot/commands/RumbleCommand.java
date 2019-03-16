package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.function.BooleanSupplier;

public class RumbleCommand extends Command {
  final double kRumbleTime = 0.300; //seconds - 300 ms to make noise
  
  BooleanSupplier boolFunc;
  XboxController ctrlr;
  GenericHID.RumbleType rumbleType = RumbleType.kRightRumble;
  double m_startTime;
  Trigger vButton;

  public RumbleCommand(XboxController ctrlr, BooleanSupplier boolFunct) {
    this.boolFunc = boolFunct;
    this.ctrlr = ctrlr; 
    initialize();

    // tie the boolFunc to a trigger 
    vButton = new RumbleTrigger(boolFunct);
    vButton.whenActive(this);
  }

  /**
   * Zeros the arm's encoders - arm and extension are at starting point
   */
  @Override
  protected void initialize() {
    setTimeout(kRumbleTime);
    ctrlr.setRumble(rumbleType, 1.0);
  }

  @Override
  protected void execute() {   }
  
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  @Override
  protected void end() {
    ctrlr.setRumble(rumbleType, 0.0);   
  }

  // Trigger to tie to this command
  public class RumbleTrigger extends Trigger {
    BooleanSupplier boolFunc;
    
    public RumbleTrigger(BooleanSupplier boolFunct) {
      this.boolFunc = boolFunct;
    }
  
    @Override
    public boolean get() {
        return boolFunc.getAsBoolean();
    }
  }
}
