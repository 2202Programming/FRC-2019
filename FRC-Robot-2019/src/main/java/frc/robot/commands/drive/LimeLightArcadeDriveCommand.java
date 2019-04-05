package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.util.ExpoShaper;
import frc.robot.input.LimeLightXFilteredInput;
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
  private ExpoShaper speedShaper;
  private double maxSpeed;

  public LimeLightArcadeDriveCommand(double maxSpeed) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    driveTrain = Robot.driveTrain;
    controller = new PIDController(P, I, D, new LimeLightXFilteredInput(), new FakePIDOutput());
    speedShaper = new ExpoShaper(0.6); // 0 no change, 1.0 max flatness
    this.maxSpeed = maxSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    controller.reset();
    controller.setInputRange(-25.0, 25.0);
    controller.setOutputRange(-0.5, 0.5);
    controller.setPercentTolerance(1);
    controller.setContinuous(true);
    controller.enable();
    Robot.sensorSubystem.enableLED();
    execute();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  protected void execute() {
    // We invert the PID controller value so the feedback loop is negative and not
    // positive
    double speed = maxSpeed * speedShaper.expo(Robot.m_oi.getDriverController().getY(Hand.kLeft));
    double rotation = -controller.get();

    if (Math.abs(rotation) <= 0.12) {
      rotation = Math.signum(rotation) * 0.12;
    }

    Robot.driveTrain.ArcadeDrive(speed, rotation, true);
    SmartDashboard.putData(controller);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.sensorSubystem.disableLED();
    controller.reset();
    driveTrain.stop();
  }
}