package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class InvertDriveControlsCommand extends InstantCommand {
    private DriveTrainSubsystem driveTrain;

    public InvertDriveControlsCommand() {
        requires(Robot.driveTrain);
        driveTrain = Robot.driveTrain;
    }

    @Override
    protected void execute() {
        driveTrain.invertControls();
        System.out.println("Controls Inverted \t Status: " + driveTrain.getInversionConstant());
    }
}