package frc.robot.commands.arm.tests;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.util.RateLimiter;
import frc.robot.commands.util.RateLimiter.InputModel;

public class TestArmRateCmd extends CommandGroup {

    RateLimiter armRC;
    RateLimiter extenderRC;

    public TestArmRateCmd() {
        armRC = new RateLimiter(Robot.dT,
                this::getShoulderCmd, 
                Robot.arm::getAngle, Robot.arm::setAngle, 
                Robot.arm.PHI_MIN, // ShoulderMinDegrees,
                Robot.arm.PHI_MAX, // ShoulderMaxDegrees,
                -3.0, // dx_falling  
                5.0, // dx_raising
                InputModel.Position); // expo


        extenderRC = new RateLimiter(Robot.dT,
            this::getExtenderCmd, 
            Robot.arm::getExtension, Robot.arm::setExtension, 
            5.0,  //Robot.arm.EXTEND_MIN, // inches,
            15.0, //Robot.arm.EXTEND_MAX, // inches
            -5.0, // dx_falling
            5.0,  // dx_raising
            InputModel.Position); 

        class RateCmd extends Command {
            final RateLimiter rc;

            RateCmd(RateLimiter _rc) {
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
        SmartDashboard.putNumber("rc:sh:cmd", armRC.get() );
        SmartDashboard.putNumber("rc:ext:cmd", extenderRC.get()); 
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