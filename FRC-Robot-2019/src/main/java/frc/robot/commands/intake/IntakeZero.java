package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class IntakeZero extends Command {

    public IntakeZero() {
        requires(Robot.intake);
    }

    
    @Override
    protected void initialize() {
        Robot.intake.setAngle(Robot.intake.WristStraightDegrees);
    }

    @Override
    protected void execute() {  }

    // This is just a test, it doesn't finish. Enjoy moving the write with the
    // controller.
    @Override
    public boolean isFinished() {
        return true;
    }

}