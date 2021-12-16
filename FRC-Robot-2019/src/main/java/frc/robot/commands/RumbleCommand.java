package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.function.BooleanSupplier;

public class RumbleCommand extends CommandBase {
  final double kRumbleTime = 2.300; //seconds - 300 ms to make noise
  
  BooleanSupplier boolFunc;
  XboxController ctrlr;
  double m_startTime;
  Trigger vButton;

  public RumbleCommand(XboxController ctrlr, BooleanSupplier boolFunct) {
    this.boolFunc = boolFunct;
    this.ctrlr = ctrlr; 

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
    ctrlr.setRumble(RumbleType.kRightRumble, 0.1);
    ctrlr.setRumble(RumbleType.kLeftRumble, 0.1);
    System.out.print("**********RUMBLE RUMBLE RUMBLE******");
  }

  @Override
  protected void execute() {   }
  
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  @Override
  protected void () {
    ctrlr.setRumble(RumbleType.kRightRumble, 0.0);
    ctrlr.setRumble(RumbleType.kLeftRumble, 0.0);   
    System.out.print("N0 RUMBLE No RUMBLE n0 RUMBLE");
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
