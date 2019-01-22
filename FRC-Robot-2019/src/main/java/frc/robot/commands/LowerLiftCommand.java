package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Lift;

public class LowerLiftCommand extends Command {
    public static final double LIFT_RELATIVE_LOWER_SETPOINT = 0.1;

    public LowerLiftCommand() {
        requires(Robot.lift);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.lift.runDown();
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