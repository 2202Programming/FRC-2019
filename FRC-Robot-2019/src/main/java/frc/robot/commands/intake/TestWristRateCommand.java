package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.util.RateController;

public class TestWristRateCommand extends Command {
    RateController wristRC;

    public TestWristRateCommand() {

        requires(Robot.intake);
        wristRC = new RateController(this::getCmd, Robot.intake::getAngle, Robot.intake::setAngle,
                Robot.intake.WristMinDegrees, 
                Robot.intake.WristMaxDegrees, 
                20.0, // dx_min deg/sec (magnitude)
                60.0, // dx_max deg/sec
                -0.15, // dz_min (normalized units)
                0.15, // dz_manx(normalized units)
                0.0); // expo
    }

    // Must supply a function to get a user's command in normalized units
    public double getCmd() {
        return Robot.m_oi.getAssistantController().getY(Hand.kLeft);
    }

    @Override
    protected void initialize() {
        wristRC.initialize();
    }

    @Override
    protected void execute() {
        wristRC.execute();
    }

    // This is just a test, it doesn't finish. Enjoy moving the write with the
    // controller.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void log() {
        SmartDashboard.putNumber("wr:cmd", wristRC.X());
    }
}