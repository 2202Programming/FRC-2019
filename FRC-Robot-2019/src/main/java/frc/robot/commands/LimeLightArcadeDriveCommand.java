package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.input.LimeLightXValueInput;
import frc.robot.output.FakePIDOutput;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class LimeLightArcadeDriveCommand extends Command {
  private DriveTrainSubsystem driveTrain;
  private final double P = 0.055;
  private final double I = 0.0;
  private final double D = 0.0;
  private PIDController controller;

  public LimeLightArcadeDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    driveTrain = Robot.driveTrain;
    controller = new PIDController(P, I, D, new LimeLightXValueInput(), new FakePIDOutput());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.stop();
    controller.reset();
    controller.setInputRange(-25.0, 25.0);
    controller.setOutputRange(-1, 1);
    controller.setPercentTolerance(1);
    controller.setContinuous(true);
    controller.enable();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  protected void execute() {
    //We invert the PID controller value so the feedback loop is negative and not positive
    Robot.driveTrain.ArcadeDrive(Robot.m_oi.getDriverController().getY(Hand.kLeft),-controller.get(), true);
    SmartDashboard.putData(controller);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    controller.reset();
    driveTrain.stop();
  }
}