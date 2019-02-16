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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SerialPortSubsystem;
import frc.robot.subsystems.GearShifterSubsystem;
import frc.robot.RobotMap;
import frc.robot.commands.arm.TestArmRateCmd;
import frc.robot.commands.intake.TestWristRateCommand;
import frc.robot.commands.CommandManager;
import frc.robot.commands.CommandManager.Modes;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //common constants for robot
  public static double dT = kDefaultPeriod;  // Robots sample period (seconds)  
  
  //physical devices and subsystems
  public static DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  public static GearShifterSubsystem gearShifter = new GearShifterSubsystem(driveTrain.kShiftPoint);
  public static IntakeSubsystem intake = new IntakeSubsystem();
  public static CargoTrapSubsystem cargoTrap = new CargoTrapSubsystem();
  public static ArmSubsystem arm = new ArmSubsystem();
  public static SerialPortSubsystem serialSubsystem = new SerialPortSubsystem();
  public static OI m_oi = new OI(); //OI Depends on the subsystems and must be last

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  CommandManager m_cmdMgr;  

  // TESTING Started in TestInit
  TestWristRateCommand testWristCmd; 
  TestArmRateCmd testArmCmd; 

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    m_cmdMgr = new CommandManager();
    m_cmdMgr.setMode(Modes.SettingZeros);   // schedules the mode's functions

    //TESTING Commands, only get scheduled if we enter Test mode
    testWristCmd = new TestWristRateCommand();
    testArmCmd = new TestArmRateCmd();
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
    m_cmdMgr.setMode(Modes.HuntingHatch);   
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    serialSubsystem.processSerial();
  }

   @Override
   public void testInit() {
     super.testInit();
     testArmCmd.start();
     testWristCmd.start();
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

    SmartDashboard.putNumber("Arm:Phi(raw)", arm.getRotationEncoder().getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm:Ext(raw)", arm.getExtensionEncoder().getSelectedSensorPosition());
    SmartDashboard.putBoolean("Arm:Ext@Min", arm.extensionAtMin());
    SmartDashboard.putBoolean("Arm:Ext@Max", arm.extensionAtMax());
    SmartDashboard.putNumber("Arm:Phi(deg)", arm.getAngle());
    SmartDashboard.putNumber("Arm:Ext(in)", arm.getExtension());
    
    testArmCmd.log();

    SmartDashboard.putNumber("Wrist(deg)", intake.getAngle());
    intake.log();   //DPL 2/10/19 review this with Billy/Xander
    arm.log();
    arm.logTalons();
    
    SmartDashboard.putData(Scheduler.getInstance()); 
    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData(gearShifter);
    SmartDashboard.putNumber("Left Front LIDAR (mm)", serialSubsystem.getDistance(RobotMap.LEFT_FRONT_LIDAR));
    SmartDashboard.putNumber("Right Front LIDAR (mm)", serialSubsystem.getDistance(RobotMap.RIGHT_FRONT_LIDAR));
  }

  private void resetAllDashBoardSensors() {
    driveTrain.getLeftEncoderTalon().setSelectedSensorPosition(0);
    driveTrain.getRightEncoderTalon().setSelectedSensorPosition(0);
  }
}
