package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.LinearFilter;
import frc.robot.Robot;
import frc.robot.commands.util.ExpoShaper;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class LimeLightArcadeDriveCommand extends Command {
  private DriveTrainSubsystem driveTrain;
  private final double P = 0.22;
  private final double I = 0.0;
  private final double D = 0.3;
  private PIDController controller;
  private ExpoShaper speedShaper;
  private double maxSpeed;
  private LinearFilter lowPass;

  //old version used 1% position tolerance, so do that - dpl 10/23/2020
  private final double POS_TOL = .005;  //TODO: units?
  private final double VEL_TOL = 0.05;  //TODO: units?

  public LimeLightArcadeDriveCommand(double maxSpeed) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    driveTrain = Robot.driveTrain;

    speedShaper = new ExpoShaper(0.6); // 0 no change, 1.0 max flatness
    lowPass = LinearFilter.movingAverage(5);
    this.maxSpeed = maxSpeed; 

    controller = new PIDController(P, I, D) ;// new LimeLightXFilteredInput(), new FakePIDOutput());
    controller.enableContinuousInput(-25.0, 25.0);   //inches??
    controller.setIntegratorRange(-0.5, 0.5);        //units??
    controller.setTolerance(POS_TOL, VEL_TOL);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    controller.reset();
    lowPass.reset();

    Robot.sensorSubystem.enableLED();
    // execute();   //dpl - 10/21/2020 don't think we need to call execute() on init()
  }

  @Override
  protected void execute() {

    //DPL - 10/25/2020 not sure what the setpoint was, are we driving to zero from the Limelight?
    double setpoint = 0.0;  //### hack

    // We invert the PID controller value so the feedback loop is negative and not
    // positive
    double speed = maxSpeed * speedShaper.expo(Robot.m_oi.getDriverController().getY(Hand.kLeft));
    double measurement = lowPass.calculate(Robot.sensorSubystem.getX());
    double rotation = -controller.calculate(measurement, setpoint);
 
    if (Math.abs(rotation) <= 0.12) {
      rotation = Math.signum(rotation) * 0.12;
    }

    // call the drive train
    Robot.driveTrain.ArcadeDrive(speed, rotation, true);

    SmartDashboard.putNumber("LLPID PosError", controller.getPositionError() );
    SmartDashboard.putNumber("LLPID VelError", controller.getVelocityError() );
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