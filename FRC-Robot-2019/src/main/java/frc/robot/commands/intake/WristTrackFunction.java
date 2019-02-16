package frc.robot.commands.intake; 
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WristTrackFunction extends Command{
    DoubleSupplier  angleFunct;

    public WristTrackFunction(DoubleSupplier angleFunct){
        requires(Robot.intake);
        this.angleFunct = angleFunct;
    }

    @Override
    protected void execute() {
        //intake angle is relative to arm
        double angle = angleFunct.getAsDouble();
        Robot.intake.setAngle(angle) ;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}