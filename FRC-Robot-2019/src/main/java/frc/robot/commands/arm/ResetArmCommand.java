package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ResetArmCommand extends InstantCommand {
  public ResetArmCommand() {
    requires(Robot.arm);
  }

  @Override
  protected void execute() {
    Robot.arm.resetArm();
  }
}
