package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

//Commands the arm to follow an arc
public class MoveArmToRawPosition extends CommandBase {
    private double endAngle;
    private double extension;
    private double curAngle;
    private double tolerance;
    private double step;
    private double degreesPerSecond;

    public MoveArmToRawPosition(double angle, double extension, double endTolerance, double degreesPerSecond) {
        addRequirements(Robot.arm);
        this.endAngle = angle;
        this.extension = extension;
        this.degreesPerSecond = degreesPerSecond;
        tolerance = endTolerance;
    }

    @Override
   public void initialize() {
        curAngle = Robot.arm.getRealAngle();
        step = Math.copySign(degreesPerSecond * Robot.kDefaultPeriod, endAngle - curAngle); // step is the # of
                                                                                            // degrees to change per
                                                                                            // cycle
    }

    @Override
    public void execute() {
        Robot.arm.setExtension(extension);
        if (Math.abs(Robot.arm.getExtension() - extension) <= 0.5 || Robot.arm.isExtensionOverrided()) {
            Robot.arm.setAngle(curAngle);

            if (step < 0) {
                curAngle = Math.max(curAngle + step, endAngle);
            } else {
                curAngle = Math.min(curAngle + step, endAngle);
            }
        }
    }

    public boolean isFinished() {
        return Math.abs(Robot.arm.getRealAngle() - endAngle) <= tolerance;
    }

    @Override
    public void end(boolean interrupted) {
    }

}