package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class DriveDistanceCommand extends InstantCommand {
    private int distanceToDrive;
    
    public DriveDistanceCommand(int distance) {
        requires(Robot.driveTrain);
        distanceToDrive = distance;
    }

    public void execute() {
        Robot.driveTrain.driveDistance(distanceToDrive);
    }
}