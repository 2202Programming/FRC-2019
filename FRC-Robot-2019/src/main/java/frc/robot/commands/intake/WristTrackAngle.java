package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class WristTrackAngle extends CommandBase {
    private DoubleSupplier angleSupplier;

    /**
     * Makes the wrist track a specific angle from vertical
     */
    public WristTrackAngle(double trackedAngle) {
        this(() -> trackedAngle);
    }

    public WristTrackAngle(DoubleSupplier trackedAngle) {
        addRequirements(Robot.intake);
        angleSupplier = trackedAngle;
    }

    @Override
    public void execute() {
        // intake angle is relative to arm
        double offset = Robot.arm.getRealAngle() - angleSupplier.getAsDouble();
        Robot.intake.setAngle(offset);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}