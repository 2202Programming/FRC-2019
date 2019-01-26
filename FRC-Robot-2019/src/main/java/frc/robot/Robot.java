/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GearShifterSubsystem;;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.%
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  public static DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  public static GearShifterSubsystem gearShifter = new GearShifterSubsystem();

   /* Hardware */
   TalonSRX _talon = new TalonSRX(11);
   Joystick _joy = new Joystick(0);

   /* String for output */
   StringBuilder _sb = new StringBuilder();

   /* Loop tracker for prints */
   int _loops = 0;
   
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    /* Factory Default all hardware to prevent unexpected behaviour */
    _talon.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
    _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
            Constants.kTimeoutMs);

    /**
     * Phase sensor accordingly. Positive Sensor Reading should match Green
     * (blinking) Leds on Talon
     */
    _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs */
    _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    _talon.config_kF(Constants.kPIDLoopIdx, 1023/2950.0, Constants.kTimeoutMs);
    _talon.config_kP(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
    _talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
    _talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    logSmartDashboardSensors();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
    resetAllDashBoardSensors();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    logSmartDashboardSensors();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    resetAllDashBoardSensors();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    /* Get gamepad axis */
    double leftYstick = -1 * _joy.getY();

    /* Get Talon/Victor's current output percentage */
    double motorOutput = _talon.getMotorOutputPercent();

    /* Prepare line to print */
    _sb.append("\tout:");
    /* Cast to int to remove decimal places */
    _sb.append((int) (motorOutput * 100));
    _sb.append("%"); // Percent

    _sb.append("\tspd:");
    _sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
    _sb.append("u"); // Native units

    /**
     * When button 1 is held, start and run Velocity Closed loop. Velocity Closed
     * Loop is controlled by joystick position x500 RPM, [-500, 500] RPM
     */
    if (_joy.getRawButton(2)) {
        /* Velocity Closed Loop */

        /**
         * Convert 500 RPM to units / 100ms. 4096 Units/Rev * 500 RPM / 600 100ms/min in
         * either direction: velocity setpoint is in units/100ms
         */
        double targetVelocity_UnitsPer100ms = leftYstick * 500.0 * 4096 / 600;
        /* 500 RPM in either direction */
        _talon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

        /* Append more signals to print when in speed mode. */
        _sb.append("\terr:");
        _sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
        _sb.append("\ttrg:");
        _sb.append(targetVelocity_UnitsPer100ms);
    } else {
        /* Percent Output */

        _talon.set(ControlMode.PercentOutput, leftYstick);
    }

    /* Print built string every 10 loops */
    if (++_loops >= 10) {
        _loops = 0;
        System.out.println(_sb.toString());
    }
    /* Reset built string */
    _sb.setLength(0);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void logSmartDashboardSensors() {
    // SmartDashboard.putNumber("Left Encoder Count", driveTrain.getLeftEncoderTalon().getSelectedSensorPosition());
    // SmartDashboard.putNumber("Left Encoder Rate", driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Right Encoder Count", driveTrain.getRightEncoderTalon().getSelectedSensorPosition());
    // SmartDashboard.putNumber("Right Encoder Rate", driveTrain.getRightEncoderTalon().getSelectedSensorVelocity());
    // SmartDashboard.putString("Gear Shifter State", String.valueOf(gearShifter.getCurGear()));
    // SmartDashboard.putData(Scheduler.getInstance()); 
    // SmartDashboard.putData(driveTrain);
    // SmartDashboard.putData(gearShifter);
  }

  private void resetAllDashBoardSensors() {
    // driveTrain.getLeftEncoderTalon().setSelectedSensorPosition(0);
    // driveTrain.getRightEncoderTalon().setSelectedSensorPosition(0);
  }
}
