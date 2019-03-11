package frc.robot.commands.intake; 
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WristTrackFunction extends Command{
    DoubleSupplier angleFunct;
    private DoubleSupplier offset;

    public WristTrackFunction(DoubleSupplier angleFunct){
        this(angleFunct, () -> 0.0);
    }

    public WristTrackFunction(DoubleSupplier angleFunct, DoubleSupplier offset){
        requires(Robot.intake);
        this.angleFunct = angleFunct;
        this.offset = offset;
    }

    @Override
    protected void execute() {
        //intake angle is relative to arm
        double angle = angleFunct.getAsDouble() + offset.getAsDouble();
        Robot.intake.setAngle(angle) ;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}