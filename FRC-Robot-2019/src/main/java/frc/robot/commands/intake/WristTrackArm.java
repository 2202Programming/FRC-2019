package frc.robot.commands.intake; 
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WristTrackArm extends Command{
    private double angle;

    public WristTrackArm(double angle){
        requires(Robot.intake);
        this.angle = angle;
    }

    @Override
    protected void execute() {
        Robot.intake.setAngle(Robot.arm.getAngle() - 90 - angle);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}