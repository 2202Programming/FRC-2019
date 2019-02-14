package frc.robot.commands.drive.outtake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
//import frc.robot.input.XboxControllerButtonCode;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends InstantCommand {
    private IntakeSubsystem intake;

    public OuttakeCommand() {
        requires(Robot.intake);
        intake = Robot.intake;
    }

    @Override
    protected void initialize() {
        intake.vacuumOff();    
    }

    @Override
    protected void interrupted() {
        intake.vacuumOff();
    }
}
