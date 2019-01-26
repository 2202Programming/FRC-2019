package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
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
        if(gearShifter.getDefaultCommandName() == "AutomaticGearShiftCommand") {
            Robot.gearShifter.setDefaultCommand(new WaitCommand(1));
        } else {
            Robot.gearShifter.setDefaultCommand(new AutomaticGearShiftCommand());
        }
    }
}