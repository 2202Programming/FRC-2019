/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.arm.ResetArmCommand;
import frc.robot.commands.cargo.AutoCargoIntakeCommand;
//import frc.robot.commands.cargo.DeployCargoTrapCommand;
//import frc.robot.commands.cargo.RetractCargoTrapCommand;
import frc.robot.commands.cargo.tests.IntakeTestCmd;
import frc.robot.commands.cargo.tests.OuttakeTestCmd;
import frc.robot.commands.climb.PawlSureFire;
import frc.robot.commands.climb.tests.CharonSolenoidTestCmd;
import frc.robot.commands.climb.tests.ClimbMotorTestCmd;
import frc.robot.commands.climb.tests.PawlSolenoidTestCmd;
import frc.robot.commands.climb.tests.RollerMotorTestCmd;
import frc.robot.commands.drive.CopilotControlCommand;
import frc.robot.commands.drive.InvertDriveControlsCommand;
import frc.robot.commands.drive.LimeLightArcadeDriveCommand;
import frc.robot.commands.drive.shift.AutomaticUpShiftCommand;
import frc.robot.commands.drive.shift.DownShiftCommand;
import frc.robot.commands.drive.shift.ToggleAutomaticGearShiftingCommand;
import frc.robot.commands.drive.shift.UpShiftCommand;
import frc.robot.commands.intake.VacuumCommand;
import frc.robot.commands.util.ExpoShaper;
import frc.robot.input.XboxControllerButtonCode;
import frc.robot.input.triggers.GeneralTrigger;
import frc.robot.input.triggers.JoystickTrigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  private XboxController driver = new XboxController(0);
  private XboxController assistant = new XboxController(1);
  private XboxController switchBoard = new XboxController(2);
  private XboxController phantom = new XboxController(3);

  // OI - operator inputs
  public JoystickButton heightDownSelect; // used in hunting/delivery modes
  public JoystickButton heightUpSelect; // used in hunting/delivery
  public JoystickButton captureRelease; // flips hunt/deliver mode
  public JoystickButton flip; // used to flip
  public JoystickButton endDriveMode; // Switches state out of drive
  public JoystickButton goToPrevMode; // Goes to previous state (only works for recapturing)

  public Button climbButton;
  public Button shortClimbButton;

  private ExpoShaper rotateShaper = new ExpoShaper(.7); // fairly flat curve

  public OI() {
      bindFieldButtons();
      
  }

  private void bindFieldButtons() {
    System.out.println("Using Field OI");
    // Drive Train Commands
    new JoystickButton(driver, XboxControllerButtonCode.B.getCode())
        .whenPressed(new ToggleAutomaticGearShiftingCommand());
    new JoystickButton(driver, XboxControllerButtonCode.X.getCode()).whenPressed(new InvertDriveControlsCommand());
    new JoystickButton(driver, XboxControllerButtonCode.LB.getCode()).whileHeld(new LimeLightArcadeDriveCommand(1.0));
    new JoystickButton(driver, XboxControllerButtonCode.RB.getCode()).whenPressed(new AutomaticUpShiftCommand());
    new JoystickTrigger(driver, XboxControllerButtonCode.TRIGGER_LEFT.getCode(), 0.75)
        .whileHeld(new AutoCargoIntakeCommand(0.4));
    new JoystickTrigger(driver, XboxControllerButtonCode.TRIGGER_RIGHT.getCode(), 0.75)
        .whileHeld(new OuttakeTestCmd(0.4));
    new JoystickButton(assistant, 9).whileHeld(new CopilotControlCommand(0.4, 0.4));;

    // Switchboard Assignments 
    
    // Climber tests - temporary added to field
    //execute Pawl only on change
    new JoystickButton(switchBoard, 1).whenPressed(new PawlSureFire(Robot.climber.Extend, 3));
    new JoystickButton(switchBoard, 1).whenReleased(new PawlSureFire(Robot.climber.Retract, 3));
    new JoystickButton(switchBoard, 2).whileActive(new ClimbMotorTestCmd(0.3));
    
    //execute Charon only on button change
    new JoystickButton(switchBoard, 3).whenPressed(new CharonSolenoidTestCmd(true));
    new JoystickButton(switchBoard, 3).whenReleased(new CharonSolenoidTestCmd(false));

    new JoystickButton(switchBoard, 4).whileActive(new RollerMotorTestCmd(0.25));
    new JoystickButton(switchBoard, 5).whileActive(new ClimbMotorTestCmd(-0.3));

    new GeneralTrigger(Robot.arm::extensionAtMin).whenPressed(new ResetArmCommand());

    // setup buttons for use in CommandManager
    heightDownSelect = new JoystickButton(assistant, XboxControllerButtonCode.LB.getCode());
    heightUpSelect = new JoystickButton(assistant, XboxControllerButtonCode.RB.getCode());
    captureRelease = new JoystickButton(assistant, XboxControllerButtonCode.A.getCode());
    flip = new JoystickButton(assistant, XboxControllerButtonCode.X.getCode());
    endDriveMode = new JoystickButton(assistant, XboxControllerButtonCode.B.getCode());
    goToPrevMode = new JoystickButton(assistant, XboxControllerButtonCode.Y.getCode());

    climbButton = new GeneralTrigger(() -> switchBoard.getRawButton(7) && switchBoard.getRawButton(11));
    shortClimbButton = new GeneralTrigger(() -> switchBoard.getRawButton(8) && switchBoard.getRawButton(11));
  }


  /**
   *  Test Bindings - used if the hard coded flag above is set to TRUE see line#.
   */
  public void bindTestButtons() {
    // Vacuum subsystem tests
    // new JoystickButton(assistant,
    // XboxControllerButtonCode.B.getCode()).whenPressed(new
    // SolenoidTestCommand(false));
    new JoystickButton(assistant, XboxControllerButtonCode.A.getCode()).whenPressed(new VacuumCommand(true, 2.0));
    new JoystickButton(assistant, XboxControllerButtonCode.B.getCode()).whenPressed(new VacuumCommand(false, 2.0));

    // gearbox tests
    new JoystickButton(driver, XboxControllerButtonCode.A.getCode()).whenPressed(new DownShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.Y.getCode()).whenPressed(new UpShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.B.getCode()).whileHeld(new IntakeTestCmd(0.4));
    new JoystickButton(driver, XboxControllerButtonCode.X.getCode()).whileHeld(new OuttakeTestCmd(0.4));

    // Climber tests
    //execute Pawl only on change
    new JoystickButton(switchBoard, 1).whenPressed(new PawlSolenoidTestCmd(true));
    new JoystickButton(switchBoard, 1).whenReleased(new PawlSolenoidTestCmd(false));
    new JoystickButton(switchBoard, 2).whileActive(new ClimbMotorTestCmd(0.3));
    
    //execute Charon only on button change
    new JoystickButton(switchBoard, 3).whenPressed(new CharonSolenoidTestCmd(true));
    new JoystickButton(switchBoard, 3).whenReleased(new CharonSolenoidTestCmd(false));

    new JoystickButton(switchBoard, 4).whileActive(new RollerMotorTestCmd(0.5));
    new JoystickButton(switchBoard, 5).whileActive(new ClimbMotorTestCmd(-0.3));

    // setup buttons - required for Control Manager construction, but not really
    // used.
    heightDownSelect = new JoystickButton(phantom, XboxControllerButtonCode.LB.getCode());
    heightUpSelect = new JoystickButton(phantom, XboxControllerButtonCode.RB.getCode());
    captureRelease = new JoystickButton(phantom, XboxControllerButtonCode.Y.getCode());
    flip = new JoystickButton(phantom, XboxControllerButtonCode.X.getCode());
    endDriveMode = new JoystickButton(phantom, XboxControllerButtonCode.B.getCode());
  }

  // Bind analog controls to functions to use by the commands
  // this way we only change it key/stick assignemnts once.

  // Use Triggers to directly make small adustments to the arm, raw stick units
  // converted in
  // the CommandManager
  public double adjustHeight() {
    return Robot.m_oi.assistant.getTriggerAxis(Hand.kLeft) - Robot.m_oi.assistant.getTriggerAxis(Hand.kRight);
  }

  public double extensionInput() {
    return Robot.m_oi.assistant.getY(Hand.kLeft);
  }

  // assistant rotation input
  public double rotationInput() {
    double in = Robot.m_oi.assistant.getY(Hand.kRight);
    double out = rotateShaper.expo(in);
    return out;
  }

  public XboxController getDriverController() {
    return driver;
  }

  public XboxController getAssistantController() {
    return assistant;
  }

  public XboxController getSwitchBoard() {
    return switchBoard;
  }
}