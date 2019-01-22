package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LiftToMax extends Command {

    public LiftToMax() {
        requires(Robot.lift);
    }
    @Override
    protected void initialize() {
        
    }
    @Override
    protected boolean isFinished() {
        return false; // TODO: Change this
    }

}