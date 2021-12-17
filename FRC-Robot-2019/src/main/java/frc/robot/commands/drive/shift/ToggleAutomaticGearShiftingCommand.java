package frc.robot.commands.drive.shift;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.GearShifterSubsystem;

public class ToggleAutomaticGearShiftingCommand extends InstantCommand {
    private GearShifterSubsystem gearShifter;

    public ToggleAutomaticGearShiftingCommand() {
        addRequirements(Robot.gearShifter);
        gearShifter = Robot.gearShifter;
    }

    @Override
    public void execute() {
        if(gearShifter.getDefaultCommand() instanceof AutomaticGearShiftCommand) {
            Robot.gearShifter.setDefaultCommand(null);
            Robot.gearShifter.autoshiftEnabled(false);
        } else {
            Robot.gearShifter.setDefaultCommand(new AutomaticGearShiftCommand());
            Robot.gearShifter.autoshiftEnabled(true);
        }
    }
}