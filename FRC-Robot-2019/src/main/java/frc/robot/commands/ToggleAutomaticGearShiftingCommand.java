package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.GearShifterSubsystem;

public class ToggleAutomaticGearShiftingCommand extends InstantCommand {
    private GearShifterSubsystem gearShifter;

    public ToggleAutomaticGearShiftingCommand() {
        requires(Robot.gearShifter);
        gearShifter = Robot.gearShifter;
    }

    @Override
    protected void execute() {
        System.out.println("Toggle Ran");
        System.out.println(gearShifter.getDefaultCommandName());
        if(gearShifter.getDefaultCommandName().equals("AutomaticGearShiftCommand")) {
            Robot.gearShifter.setDefaultCommand(null);
        } else {
            Robot.gearShifter.setDefaultCommand(new AutomaticGearShiftCommand());
        }
    }
}