package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class IntakeZero extends CommandBase {

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