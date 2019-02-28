package frc.robot.commands.arm.tests;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TestRotateArmToAngleCommand extends Command {
    final private double kTolerance = 2.0;
    private double angle;
    private double timeout;

    public TestRotateArmToAngleCommand(double angle, double timeout) {
        this.angle = angle;
        this.timeout = timeout;
        requires(Robot.arm);
    }
    
    public TestRotateArmToAngleCommand(double angle)
    {
        this(angle, 0.0);
    }

    @Override
    protected void initialize() {
        setTimeout(timeout);
    }
    @Override
    protected void execute() {
        Robot.arm.setAngle(angle);
    }

    protected boolean isFinished() {
        boolean pos = Math.abs(Robot.arm.getAbsoluteAngle() - angle) < kTolerance; 
        return pos || isTimedOut();  
    }
}