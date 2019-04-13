package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

//Commands the arm to follow an arc
public class MoveArmToRawPosition extends Command {
    private double endAngle;
    private double extension;
    private double curAngle;
    private double tolerance;
    private double step;
    private double degreesPerSecond;

    public MoveArmToRawPosition(double angle, double extension, double endTolerance, double degreesPerSecond) {
        requires(Robot.arm);
        this.endAngle = angle;
        this.extension = extension;
        this.degreesPerSecond = degreesPerSecond;
        tolerance = endTolerance;
    }

    @Override
    protected void initialize() {
        System.out.println("MoveArmToRawPosition EndAngle: " + endAngle);
        curAngle = Robot.arm.getRealAngle();
        step = Math.copySign(degreesPerSecond * Robot.kDefaultPeriod, endAngle - curAngle); // step is the # of
                                                                                            // degrees to change per
                                                                                            // cycle
        System.out.println("MoveArmToRawPosition Step: " + step);
    }

    @Override
    protected void execute() {
        Robot.arm.setExtension(extension);
        if (Math.abs(Robot.arm.getExtension() - extension) <= 0.5 || Robot.arm.isExtensionOverrided()) {
            System.out.println("MoveArmToRawPosition Angle: " + curAngle);
            Robot.arm.setAngle(curAngle);

            if (step < 0) {
                curAngle = Math.max(curAngle + step, endAngle);
            } else {
                curAngle = Math.min(curAngle + step, endAngle);
            }
            System.out.println("MoveArmToRawPosition New Angle: " + curAngle);
        }
    }

    protected boolean isFinished() {
        return Math.abs(Robot.arm.getRealAngle() - endAngle) <= tolerance;
    }

    @Override
    protected void end() {
    }

}