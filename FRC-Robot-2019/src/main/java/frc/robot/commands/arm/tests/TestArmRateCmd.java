package frc.robot.commands.arm.tests;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
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
                Robot.arm::getAbsoluteAngle, 
                Robot.arm.PHI_MIN, // ShoulderMinDegrees,
                Robot.arm.PHI_MAX, // ShoulderMaxDegrees,
                -3.0, // dx_falling  
                5.0, // dx_raising
                InputModel.Position); // expo


        extenderRC = new RateLimiter(Robot.dT,
            this::getExtenderCmd, 
            Robot.arm::getExtension,
            5.0,  //Robot.arm.EXTEND_MIN, // inches,
            15.0, //Robot.arm.EXTEND_MAX, // inches
            -5.0, // dx_falling
            5.0,  // dx_raising
            InputModel.Position); 

        class RateCmd extends Command {
            final RateLimiter rc;
            final DoubleConsumer outfunct;

            RateCmd(RateLimiter _rc, DoubleConsumer _outfunct) {
                requires(Robot.arm);
                rc = _rc;
                outfunct = _outfunct;
            }

            @Override
            protected void initialize() { rc.initialize();   }

            @Override
            protected void execute() { 
                rc.execute();
                outfunct.accept(rc.get());
            }

            @Override
            public boolean isFinished() { return false;  }
        }

        RateCmd shoulderCmd = new RateCmd(armRC, Robot.arm::setAngle);
        RateCmd extenderCmd = new RateCmd(extenderRC, Robot.arm::setExtension);
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