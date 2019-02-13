
package frc.robot.commands.intake; 

import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Robot;
import frc.robot.commands.util.RateCommand;

public class TestWristRateCommand extends RateCommand {

    public TestWristRateCommand( ) {
        super(Robot.intake, 
            
            Robot.intake::getAngle,
            Robot.intake::setAngle,
            Robot.intake.WristMinDegrees,
            Robot.intake.WristMaxDegrees,
             20.0,  //dx_min deg/sec (magnitude)
             60.0,  //dx_max deg/sec
            -0.15,  //dz_min (normalized units)
             0.15,  //dz_manx(normalized units)
             0.0);   // expo
        
    }
    @Override
    public double getCmd() {
        return Robot.m_oi.getAssistantController().getY(Hand.kLeft);
    }
}