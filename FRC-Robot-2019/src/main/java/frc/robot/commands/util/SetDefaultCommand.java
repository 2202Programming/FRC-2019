package frc.robot.commands.util; 

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SetDefaultCommand extends InstantCommand{
    private Subsystem system;
    private Command defaultCommand;
    /**
     * A command to change the default command of a subsystem.
     */
    public SetDefaultCommand(Subsystem system, Command newDefault){
        addRequirements(system);
        this.system = system;
        defaultCommand = newDefault;
    }

    @Override
    public void execute() {
        system.setDefaultCommand(defaultCommand);
        System.out.println("Set Default Command to: " + defaultCommand.getName());
    }
}