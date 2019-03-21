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
    protected void initialize() {
        Robot.intake.setDefaultCommand(this);
    }

    @Override
    protected void execute() {
        // intake angle is relative to arm
        double offset = angleSupplier.getAsDouble() - Robot.arm.getRealAngle();
        System.out.println("Wrist Angle: " + offset);
        Robot.intake.setAngle(offset);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    public enum Angle {
        Perpendicular_Up(0.0), Cargo_Delivery(60.0), Starting_Hatch_Hunt(76.0), Hatch_Delivery(83.0), Parallel(90.0),
        Perpendicular_Down(180.0);

        private double phi;

        Angle(double phi) {
            this.phi = phi;
        }

        public double getAngle() {
            return phi;
        }
    }
}