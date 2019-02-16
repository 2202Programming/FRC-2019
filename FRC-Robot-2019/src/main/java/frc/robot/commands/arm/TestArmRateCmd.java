package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.util.RateController;

public class TestArmRateCmd extends CommandGroup {

    RateController armRC;
    RateController extenderRC;

    public TestArmRateCmd() {
        armRC = new RateController(this::getShoulderCmd, 
                Robot.arm::getAngle, Robot.arm::setAngle, 
                Robot.arm.PHI_MIN, // ShoulderMinDegrees,
                Robot.arm.PHI_MAX, // ShoulderMaxDegrees,
                2.0, // dx_min deg/sec (magnitude)
                2.0, // dx_max deg/sec
                -0.15, // dz_min (normalized units)
                0.15, // dz_manx(normalized units)
                0.0); // expo


        extenderRC = new RateController(this::getExtenderCmd, 
            Robot.arm::getExtension, Robot.arm::setExtension, 
            0.0, //Robot.arm.EXTEND_MIN, // inches,
            15.0, //Robot.arm.EXTEND_MAX, // inches
            5.0, // dx_min in/sec (magnitude)
            5.0, // dx_max in/sec
           -0.05, // dz_min (normalized units)
            0.05, // dz_manx(normalized units)
            0.0); // expo

        class RateCmd extends Command {
            final RateController rc;

            RateCmd(RateController _rc) {
                requires(Robot.arm);
                rc = _rc;
            }

            @Override
            protected void initialize() { rc.initialize();   }

            @Override
            protected void execute() { rc.execute();   }

            @Override
            public boolean isFinished() { return false;  }
        }

        RateCmd shoulderCmd = new RateCmd(armRC);
        RateCmd extenderCmd = new RateCmd(extenderRC);
        addParallel(shoulderCmd);
        addParallel(extenderCmd);
    }

    public void log() {
        SmartDashboard.putNumber("rc:sh:cmd", armRC.X() );
        SmartDashboard.putNumber("rc:ext:cmd", extenderRC.X()); 
    }
    // ### TODO: move the binding functions to a less hidden place - DPL
    // Bind the control to our functions
    public double getShoulderCmd() {
        return Robot.m_oi.getAssistantController().getY(Hand.kRight);
    }
    public double getExtenderCmd() {
        return Robot.m_oi.getAssistantController().getX(Hand.kRight);
    }
}