/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CommandManager;
import frc.robot.commands.CommandManager.Modes;
import frc.robot.commands.arm.MoveArmAtHeight;
import frc.robot.commands.climb.CheckSolenoids;
import frc.robot.commands.climb.ClimbGroup;
import frc.robot.commands.climb.ClimbUpPartial;
import frc.robot.commands.climb.PullUpPartial;
import frc.robot.commands.intake.CheckSucc;
import frc.robot.commands.intake.WristTrackFunction;
import frc.robot.commands.util.CancelCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CargoTrapSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GearShifterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;

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
  //THis years bounding box beyond frame of robot. Use this in limit calcs in subsystems.
  public static double kProjectConstraint = 26.0; //inches from frame (accounting for the suction cup length)
  //public static double kForwardProjectMin = 18.0; //inches from arm pivot x-axis to bumper
  //public static double kReverseProjectMin = 18.0; //inches from arm pivot x-axis to bumper
  
  //physical devices and subsystems
  public static DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  public static GearShifterSubsystem gearShifter = new GearShifterSubsystem(driveTrain.kShiftPoint);
  public static IntakeSubsystem intake = new IntakeSubsystem();
  public static CargoTrapSubsystem cargoTrap = new CargoTrapSubsystem();
  public static ArmSubsystem arm = new ArmSubsystem();
  public static ClimberSubsystem climber = new ClimberSubsystem();
  public static PowerDistributionPanel pdp = new PowerDistributionPanel(0);
  public static CameraSubsystem cameraSubsystem = new CameraSubsystem();
  public static SensorSubsystem sensorSubystem = new SensorSubsystem();

  public static OI m_oi = new OI(); //OI Depends on the subsystems and must be last (boolean is whether we are testing or not)

  public static CommandManager m_cmdMgr;    //fix the public later
  private RobotTest m_testRobot;

  boolean doneOnce = false;   //single execute our zero 
  private Integer currentCamera = 1;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Create the test subsystem
    m_testRobot  = new RobotTest();
    m_cmdMgr = new CommandManager();
    m_cmdMgr.setMode(Modes.Construction);   // schedules the mode's function
    sensorSubystem.disableLED(); //disable blinding green LED that Trevor hates
    NetworkTableEntry cameraSelect = NetworkTableInstance.getDefault().getEntry("/PiSwitch");
    // 0=front cam, 1= rear cam, 2 = arm  (pi camera server defines this - could change)
    cameraSelect.setDouble(1);    
    m_cmdMgr.setMode(Modes.SettingZeros);   // schedules the mode's function    
    CommandGroup level3Climb = new ClimbGroup(25.5, 0.0);  // extend/retract in inches
    m_oi.climbButton.whenPressed(level3Climb);
    m_oi.climbButton.whenReleased(new CancelCommand(level3Climb));
    CommandGroup level2Climb = new ClimbGroup(9.0, 0.0);
    m_oi.shortClimbButton.whenPressed(level2Climb);
    m_oi.shortClimbButton.whenReleased(new CancelCommand(level2Climb));
    CommandGroup level3Up = new ClimbUpPartial(25.5, 0);
    m_oi.climbUp.whenPressed(level3Up);
    m_oi.climbUp.whenReleased(new CancelCommand(level3Up));
    CommandGroup level3Retract = new PullUpPartial(25.5, 0);
    m_oi.climbUp.whenPressed(level3Retract);
    m_oi.climbUp.whenReleased(new CancelCommand(level3Retract));
    Robot.arm.setDefaultCommand(new MoveArmAtHeight(m_cmdMgr::gripperHeightOut, m_cmdMgr::gripperXProjectionOut));
    Robot.intake.setDefaultCommand(new WristTrackFunction(m_cmdMgr::wristTrackParallel));
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
    logSmartDashboardSensors(500); //call smartdashboard logging, 500ms update rate
    sensorSubystem.processSensors();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    sensorSubystem.disableLED(); //disable blinding green LED that Trevor hates
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    sensorSubystem.disableLED(); //disable blinding green LED that Trevor hates
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
    resetAllDashBoardSensors();
    Scheduler.getInstance().add(new CheckSolenoids());

    if(!doneOnce) {
      m_cmdMgr.setMode(Modes.HuntGameStart);   // schedules the mode's function
      doneOnce = true;
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    m_cmdMgr.execute();
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    resetAllDashBoardSensors();
    if(!doneOnce) {
      m_cmdMgr.setMode(Modes.HuntGameStart);   // schedules the mode's function
      doneOnce = true;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_cmdMgr.execute();
    Scheduler.getInstance().run();
  }

   @Override
   public void testInit() {
     m_testRobot.initialize();
     sensorSubystem.disableLED(); //active limelight LED when operational
     Scheduler.getInstance().enable();   //### hack? or required?  Seems required otherwise nothing runs 
   }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
    m_testRobot.periodic();
  }

  private void logSmartDashboardSensors(int interval) {
    //calls subsystem smartdashboard logging functions, instructs them to only update every interval # of ms
    
    //picking hopefully non-overlapping time intervals so all the logging isn't done at the same cycle
    sensorSubystem.log(interval); //tell limelight to post to dashboard every Xms
    driveTrain.log(interval+3); //tell drivertrain to post to dashboard every Xms
    //serialSubsystem.log(interval+7); //tell serial to post to dashboard every Xms
    arm.log(interval+11);
    gearShifter.log(interval+17); //tell gearshifter to post to dashboard every Xms
    m_cmdMgr.log(interval+23);
    intake.log(interval+29);
    climber.log(interval+31);

    
    SmartDashboard.putData(Scheduler.getInstance()); 
    SmartDashboard.putData(arm);
    SmartDashboard.putData(intake);
}

  private void resetAllDashBoardSensors() {
    driveTrain.getLeftEncoderTalon().setSelectedSensorPosition(0);
    driveTrain.getRightEncoderTalon().setSelectedSensorPosition(0);
  }
}
