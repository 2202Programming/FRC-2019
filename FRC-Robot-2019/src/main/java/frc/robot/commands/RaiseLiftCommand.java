package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Lift;

public class RaiseLiftCommand extends Command {
    public static final double LIFT_RELATIVE_RAISE_SETPOINT = 0.1;

    public RaiseLiftCommand() {
        requires(Robot.lift);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.lift.runUp();
    }

    @Override
    protected void end() {
        Robot.lift.stop();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}