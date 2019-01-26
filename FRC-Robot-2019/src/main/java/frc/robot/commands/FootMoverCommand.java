package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class FootMoverCommand extends Command
{
    

    @Override
    protected boolean isFinished()
    {

    }

    @Override
    protected void end()
    {
        Robot.climber.stopFoot();
    }
}