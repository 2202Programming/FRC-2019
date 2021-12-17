package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class InvertDriveControlsCommand extends InstantCommand {
    private DriveTrainSubsystem driveTrain;

    public InvertDriveControlsCommand() {
        addRequirements(Robot.driveTrain);
        driveTrain = Robot.driveTrain;
    }

    @Override
    public void execute() {
        driveTrain.invertControls();
    }
}