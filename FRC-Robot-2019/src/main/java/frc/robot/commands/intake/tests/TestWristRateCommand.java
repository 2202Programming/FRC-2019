package frc.robot.commands.intake.tests;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.util.RateLimiter;
import frc.robot.commands.util.RateLimiter.InputModel;

public class TestWristRateCommand extends CommandBase {
    RateLimiter wristRC;

    public TestWristRateCommand() {
        addRequirements(Robot.intake);
        wristRC = new RateLimiter(Robot.dT,
                this::getCmd, 
                Robot.intake::getAngle, 
                Robot.intake.WristMinDegrees -10, 
                Robot.intake.WristMaxDegrees+ 10, 
                -80.0, // dx_fall deg/sec 
                180.0,  // dx_raise deg/ses
                InputModel.Rate);
        
        //finish up scaling for rate
        wristRC.setRateGain(200.0);   // stick (-1, 1) *k = deg/sec comm  -/+200 deg/sec
        wristRC.setDeadZone(5.0);    // ignore rates less than 5.0 deg/sec
    }

    // Must supply a function to get a user's command in normalized units
    public double getCmd() {
        double   temp =  Robot.m_oi.getAssistantController().getLeftY();
        return temp;
    }
    
   public void initialize() {
        wristRC.initialize();
    }

    
    public void execute() {
        wristRC.execute();
        Robot.intake.setAngle(wristRC.get());
    }

    // This is just a test, it doesn't finish. Enjoy moving the write with the
    // controller.
    
    public boolean isFinished() {
        return false;
    }

    public void log() {
        SmartDashboard.putNumber("TWR:RCout", wristRC.get());
    }
}