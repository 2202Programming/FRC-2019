package frc.robot.commands.intake.tests;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.util.RateLimiter;
import frc.robot.commands.util.RateLimiter.InputModel;

public class TestWristRateCommand extends Command {
    RateLimiter wristRC;

    public TestWristRateCommand() {
        requires(Robot.intake);
        wristRC = new RateLimiter(Robot.dT,
                this::getCmd, 
                Robot.intake::getAngle, 
                Robot.intake::setAngle,
                Robot.intake.WristMinDegrees, 
                Robot.intake.WristMaxDegrees, 
                -20.0, // dx_fall deg/sec 
                60.0,  // dx_raise deg/ses
                InputModel.Rate);
        
        //finish up scaling for rate
        wristRC.setRateGain(20.0);   // stick (-1, 1) *k = deg/sec comm  -/+20 deg/sec
        wristRC.setDeadZone(2.0);    // ignore rates less than 2.0 deg/sec
    }

    // Must supply a function to get a user's command in normalized units
    public double getCmd() {
        double   temp =  Robot.m_oi.getAssistantController().getY(Hand.kLeft);
        return temp;
    }
    
    protected void initialize() {
        wristRC.initialize();
    }

    
    protected void execute() {
        wristRC.execute();
    }

    // This is just a test, it doesn't finish. Enjoy moving the write with the
    // controller.
    
    public boolean isFinished() {
        return false;
    }

    public void log() {
        SmartDashboard.putNumber("TRR:RCout", wristRC.get());
    }
}