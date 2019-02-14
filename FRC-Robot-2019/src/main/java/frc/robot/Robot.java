/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CargoTrapSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GearShifterSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SerialPortSubsystem;
import frc.robot.RobotMap;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  public static GearShifterSubsystem gearShifter = new GearShifterSubsystem();
  public static LimeLightSubsystem limeLight = new LimeLightSubsystem();
  public static IntakeSubsystem intake = new IntakeSubsystem();
  public static CargoTrapSubsystem cargoTrap = new CargoTrapSubsystem();
  public static ArmSubsystem arm = new ArmSubsystem();
  public static OI m_oi = new OI();
  public static SerialPortSubsystem serialSubsystem = new SerialPortSubsystem();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
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
    limeLight.populateLimelight();
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
    intake.setWristPosition(0.5);
    }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    serialSubsystem.processSerial();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void logSmartDashboardSensors() {
    SmartDashboard.putNumber("Left Encoder Count", driveTrain.getLeftEncoderTalon().getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Encoder Rate", driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Encoder Count", driveTrain.getRightEncoderTalon().getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder Rate", driveTrain.getRightEncoderTalon().getSelectedSensorVelocity());
    SmartDashboard.putString("Gear Shifter State", String.valueOf(gearShifter.getCurGear()));
    SmartDashboard.putNumber("Arm Rotation Count", arm.getRotationEncoder().getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Extension Count", arm.getExtensionEncoder().getSelectedSensorPosition());
    SmartDashboard.putBoolean("Arm Extension At Min", arm.extensionAtMin());
    SmartDashboard.putBoolean("Arm Extension At Max", arm.extensionAtMax());
    SmartDashboard.putNumber("Arm Angle", arm.getAngle());
    SmartDashboard.putNumber("Arm Extension Distance", arm.getDistanceExtended());
    SmartDashboard.putNumber("Wrist Position", intake.getWristPosition());
    SmartDashboard.putNumber("Wrist Angle", intake.getWristAngle());
    intake.logWrist();
    arm.logArmRotation();
    arm.logArmExtnension();
    
    SmartDashboard.putData(Scheduler.getInstance()); 
    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData(gearShifter);
    
    SmartDashboard.putNumber("LimelightX", limeLight.getX());
    SmartDashboard.putNumber("LimelightY", limeLight.getY());
    SmartDashboard.putNumber("LimelightArea", limeLight.getArea());
    SmartDashboard.putBoolean("LimeTarget", limeLight.hasTarget());
    SmartDashboard.putNumber("Left Front LIDAR (mm)", serialSubsystem.getDistance(RobotMap.LEFT_FRONT_LIDAR));
    SmartDashboard.putNumber("Right Front LIDAR (mm)", serialSubsystem.getDistance(RobotMap.RIGHT_FRONT_LIDAR));

    //TODO: Create Lift instance field and then call LogLift();
  }

  private void resetAllDashBoardSensors() {
    driveTrain.getLeftEncoderTalon().setSelectedSensorPosition(0);
    driveTrain.getRightEncoderTalon().setSelectedSensorPosition(0);
  }

}
