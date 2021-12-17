package frc.robot.commands.arm.tests;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class TestRotateArmToAngleCommand extends WaitCommand {
    final private double kTolerance = 2.0;
    private double angle;

    public TestRotateArmToAngleCommand(double angle, double timeout) {
        super(timeout);
        this.angle = angle;
        addRequirements(Robot.arm);
    }
    
    public TestRotateArmToAngleCommand(double angle)
    {
        this(angle, 10);
    }


    @Override
    public void execute() {
        Robot.arm.setAngle(angle);
    }

    @Override
    public boolean isFinished() {
        boolean pos = Math.abs(Robot.arm.getRealAngle() - angle) < kTolerance; 
        return pos || super.isFinished();  
    }
}