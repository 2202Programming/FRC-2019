package frc.robot.commands.intake.tests;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.util.RateLimiter;
import frc.robot.commands.util.RateLimiter.InputModel;

public class TestWristPositionCommand extends Command {
    RateLimiter wristPC;

    public TestWristPositionCommand(DoubleSupplier getter) {
        requires(Robot.intake);
        wristPC = new RateLimiter(Robot.dT,
                getter, 
                Robot.intake::getAngle, 
                Robot.intake::setAngle,
                Robot.intake.WristMinDegrees, 
                Robot.intake.WristMaxDegrees, 
               -60.0,   // dx_fall deg/sec 
                180.0,  // dx_raise deg/se
                InputModel.Position);
        
        //finish up scaling for rate
        wristPC.setRateGain(100.0);   // trigger (-1, 1) *k = deg command
        wristPC.setDeadZone(10.0);    // ignore position less than 10.0 deg
    }

    
    protected void initialize() {
        wristPC.initialize();
    }

    protected void execute() {
        wristPC.execute();
    }

    // This is just a test, it doesn't finish. Enjoy moving the wrist with the controller.
    public boolean isFinished() {
        return false;
    }

    public void log() {
        SmartDashboard.putNumber("TWP:RPout", wristPC.get());
    }
}