package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.CommandManager;
import frc.robot.commands.arm.ArmStatePositioner;
import frc.robot.subsystems.VacuumSensorSystem;

public class RetractOnReleaseCommand extends WaitCommand { 
  VacuumSensorSystem vs;
  BooleanSupplier releaseCheckFunc;
  double x_retract;
  double init_x;
  double init_h;
  double x_new; // x to jump to when the sensor says we are released

  private ArmStatePositioner armPositioner;

  public RetractOnReleaseCommand(CommandManager cmdMgr, double x_retract, double timeout) {
    super(timeout);
    vs = Robot.intake.getVacuumSensor();
    releaseCheckFunc = (vs !=null) ? vs::hasReleased : this::nosensor;
    this.x_retract = x_retract;
  }

  // Called just before this Command runs the first time
  @Override
 public void initialize() {
    // Assume that the ArmStatePositioner is the only type of default command used
    armPositioner = Robot.arm.getArmPositioner();

    // save were we are so we can tweek it on finish
    init_x = armPositioner.getProjectionCommanded();
    init_h = armPositioner.getHeightCommanded();
    x_new = init_x;
    int invertMultiplier = Robot.arm.isInverted() ? -1 : 1;
    x_new -= invertMultiplier * x_retract; // move back a bit, account for side.

    super.initialize();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // DEBUG CODE
    boolean rc = releaseCheckFunc.getAsBoolean();
    SmartDashboard.putBoolean("ReleaseSensor", rc);
    // we are done, we timed out or we got the vacuum release signal, move us back.
    // RC is good, move arm back now
    if (rc) {
      armPositioner.getDriverAdjustLimiter().setX(0.0);
      armPositioner.setPosition(init_h, x_new);
    }
  }

  boolean checkArmPos() {
    double curX_proj_err = Math.abs(Robot.arm.getProjection() - x_new);
    return (curX_proj_err < .25) || Robot.arm.isExtensionOverrided(); // .25 inch
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return super.isFinished() || checkArmPos();
  }

  boolean nosensor() { return false; }

}
