package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class ResetArmCommand extends InstantCommand {
  public ResetArmCommand() {
    addRequirements(Robot.arm);
  }

  @Override
  public void execute() {
    Robot.arm.resetArm(0.0);
  }
}
