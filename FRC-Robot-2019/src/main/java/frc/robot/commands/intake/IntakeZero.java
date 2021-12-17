package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class IntakeZero extends CommandBase {

    public IntakeZero() {
        addRequirements(Robot.intake);
    }

    
    @Override
   public void initialize() {
        Robot.intake.setAngle(Robot.intake.WristStraightDegrees);
    }

    @Override
    public void execute() {  }

    // This is just a test, it doesn't finish. Enjoy moving the write with the
    // controller.
    @Override
    public boolean isFinished() {
        return true;
    }

}