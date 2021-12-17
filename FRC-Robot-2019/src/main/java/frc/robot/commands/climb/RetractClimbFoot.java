package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RetractClimbFoot extends CommandBase {
    public RetractClimbFoot() {
        addRequirements(Robot.climber);
    }

    public void execute() {
        Robot.climber.setExtenderSpeed(-0.3);
    }

    public void end(boolean interrupted) {
        Robot.climber.setExtenderSpeed(0);
    }

    public boolean isFinished() {
        return Robot.climber.getExtension() <= 10; //TODO: Better stop at bottom
    }
}