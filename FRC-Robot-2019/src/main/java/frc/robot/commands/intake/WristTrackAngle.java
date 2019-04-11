package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WristTrackAngle extends Command {
    private DoubleSupplier angleSupplier;

    /**
     * Makes the wrist track a specific angle from vertical
     */
    public WristTrackAngle(double trackedAngle) {
        this(() -> trackedAngle);
    }

    public WristTrackAngle(DoubleSupplier trackedAngle) {
        requires(Robot.intake);
        angleSupplier = trackedAngle;
    }

    @Override
    protected void execute() {
        // intake angle is relative to arm
        double offset = Robot.arm.getRealAngle() - angleSupplier.getAsDouble();
        Robot.intake.setAngle(offset);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}