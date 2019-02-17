package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class StopWristCommand extends InstantCommand {
    private double targetAngle;
    private IntakeSubsystem wrist;

    public StopWristCommand() {
        requires(Robot.intake);
    }

    @Override
    protected void execute() {
        wrist.stopWrist();
    }
}