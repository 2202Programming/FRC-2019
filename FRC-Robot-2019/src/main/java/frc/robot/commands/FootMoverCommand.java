package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Robot;

public class FootMoverCommand extends Command
{
    private Encoder moveEncoder = Robot.climber.getEncoder();//775 motor checker for distance

    public FootMoverCommand()
    {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.climber);
    }

    @Override
    protected boolean isFinished()
    {
        //This needs to be changed depending on requirements
        if(moveEncoder.getDistance() >= 5.0/*fake value*/)
            return true;
        return false;
    }


    @Override
    protected void initialize()
    {
        Robot.climber.stopFoot();
    }
    @Override
    protected void execute()
    {
        Robot.climber.moveFoot();
    }
    @Override
    protected void end()
    {
        Robot.climber.stopFoot();
    }
}