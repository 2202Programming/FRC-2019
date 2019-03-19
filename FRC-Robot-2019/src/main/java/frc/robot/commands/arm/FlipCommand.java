package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

//Commands the arm to follow an arc
public class FlipCommand extends Command {
    private double startAngle;
    private double endAngle; 
    private double extension;
    private double curAngle;
    private double tolerance;
    private double step;

    public FlipCommand(double startAngle, double endAngle, double extension, double endTolerance) {
        requires(Robot.arm);
        this.startAngle = startAngle;
        this.endAngle = endAngle;
        this.extension = extension;
        tolerance = endTolerance;
    }

    @Override
    protected void initialize() {
        curAngle = startAngle;
        step = Math.signum(startAngle - endAngle);
        execute();
    }

    @Override
    protected void execute() {
        Robot.arm.setExtension(extension);
        Robot.arm.setAngle(curAngle);
        if(step < 0) {
            curAngle = Math.max(curAngle + step, endAngle);
        } else {
            curAngle = Math.min(curAngle + step, endAngle);
        }
    }

    protected boolean isFinished() {
        return Math.abs(Robot.arm.getRealAngle() - endAngle) <= tolerance;
    }
}