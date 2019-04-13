package frc.robot.commands.util; 

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

public class SetDefaultCommand extends InstantCommand{
    private Subsystem system;
    private Command defaultCommand;
    /**
     * Makes the wrist track a specific angle from vertical
     */
    public SetDefaultCommand(Subsystem system, Command newDefault){
        requires(system);
        this.system = system;
        defaultCommand = newDefault;
    }

    @Override
    protected void execute() {
        system.setDefaultCommand(defaultCommand);
        System.out.println("Set Default Command to: " + defaultCommand.getName());
    }
}