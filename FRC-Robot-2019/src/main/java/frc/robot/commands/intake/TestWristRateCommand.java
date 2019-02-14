package frc.robot.commands.intake; 

import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Robot;
import frc.robot.commands.util.RateCommand;

public class TestWristRateCommand extends Command {
    RateCommand wristCtrl;
    public TestWristRateCommand( ) {
        requires(Robot.intake);
        wristCtrl= new RateCommand (
            this::getCmd,
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
    // Must supply a function to get a user's command in normalized units
    public double getCmd() {
        return Robot.m_oi.getAssistantController().getY(Hand.kLeft);
    }

    @Override
    protected void initialize() {
        wristCtrl.initialize();
    }

    @Override
    protected void execute() {
        wristCtrl.execute();
    }
    //This is just a test, it doesn't finish. Enjoy moving the write with the controller.
    @Override
    public boolean isFinished() {
        return false;
    }
}