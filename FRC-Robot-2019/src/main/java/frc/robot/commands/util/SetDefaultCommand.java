package frc.robot.commands.util; 

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

public class SetDefaultCommand extends InstantCommand{
    private Subsystem system;
    private Command defaultCommand;
    /**
     * Makes the wrist track a specific angle from vertical
     */
    public SetDefaultCommand(Subsystem system, Command newDefault){
        requires(system);
    }

    @Override
    protected void execute() {
        system.setDefaultCommand(defaultCommand);
    }
}