package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

public class WristSetAngleCommand extends TimedCommand {
    private double angle;

    /**
     * Sets the wrist to a specific angle
     */
    public WristSetAngleCommand(double angle) {
        this(angle, 0.0);
    }

    public WristSetAngleCommand(double angle, double timeout) {
        super(timeout);
        requires(Robot.intake);
        this.angle = angle;
    }

    @Override
    protected void initialize() {
        execute();
    }

    @Override
    protected void execute() {
        // intake angle is relative to servo
        Robot.intake.setAngle(angle);
    }
}