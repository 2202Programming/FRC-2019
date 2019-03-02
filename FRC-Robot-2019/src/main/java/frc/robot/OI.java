/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.input.XboxControllerButtonCode;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.LimeLightArcadeDriveCommand;
import frc.robot.commands.climb.tests.ClimbSolenoidTestCmd;
import frc.robot.commands.drive.*;
import frc.robot.commands.drive.shift.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.intake.tests.IntakeTestCommand;
import frc.robot.commands.intake.tests.SolenoidTestCommand;
import frc.robot.commands.intake.tests.VacuumTestCommand;
import frc.robot.commands.util.ExpoShaper;

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

  // OI - operator inputs
  public JoystickButton heightDownSelect;   // used in hunting/delivery modes
  public JoystickButton heightUpSelect;     // used in hunting/delivery
  public JoystickButton captureRelease;     // flips hunt/deliver mode
  public JoystickButton flip;               // used to flip

  private ExpoShaper rotateShaper = new ExpoShaper(.7);    //fairly flat curve


  @SuppressWarnings({ "resource", })
  public OI(boolean isTesting) {
    if(isTesting) {
      bindTestButtons();
    } else {
      bindFieldButtons();
    }
  }

  private void bindFieldButtons() {
    // Drive Train Commands
    new JoystickButton(driver, XboxControllerButtonCode.A.getCode()).whenPressed(new DownShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.Y.getCode()).whenPressed(new UpShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.B.getCode()).whenPressed(new ToggleAutomaticGearShiftingCommand());
    new JoystickButton(driver, XboxControllerButtonCode.X.getCode()).whenPressed(new InvertDriveControlsCommand());
    new JoystickButton(driver, XboxControllerButtonCode.LB.getCode()).whileHeld(new TankDriveCommand());
    new JoystickButton(driver, XboxControllerButtonCode.RB.getCode()).whileHeld(new LimeLightArcadeDriveCommand());    

    // setup buttons for use in CommandManager
    heightDownSelect = new JoystickButton(assistant, XboxControllerButtonCode.LB.getCode());
    heightUpSelect   = new JoystickButton(assistant, XboxControllerButtonCode.RB.getCode());
    captureRelease   = new JoystickButton(assistant, XboxControllerButtonCode.A.getCode());
    flip           = new JoystickButton(assistant, XboxControllerButtonCode.X.getCode());

    //Intake Commands
    //hack new JoystickButton(assistant, XboxControllerButtonCode.B.getCode()).whenPressed(new VacuumCommand(false));
    //hack new JoystickButton(assistant, XboxControllerButtonCode.A.getCode()).whenPressed(new VacuumCommand(true));
    //new JoystickButton(assistant, XboxControllerButtonCode.START.getCode()).whenPressed(new RotateWristUpCommand());
    //new JoystickButton(assistant, XboxControllerButtonCode.BACK.getCode()).whenPressed(new RotateWristDownCommand());

    //Driver assist commands (macros)
  }

  public void bindTestButtons() {
    //Vacuum subsystem tests
    new JoystickButton(assistant, XboxControllerButtonCode.A.getCode()).whenPressed(new IntakeTestCommand(false));
    //new JoystickButton(assistant, XboxControllerButtonCode.B.getCode()).whenPressed(new SolenoidTestCommand(false));
    new JoystickButton(assistant, XboxControllerButtonCode.X.getCode()).whenPressed(new VacuumTestCommand(false));

    //Climber solenoid test
    new JoystickButton(assistant, XboxControllerButtonCode.B.getCode()).whenPressed(new ClimbSolenoidTestCmd(false));

    //gearbox tests
    new JoystickButton(driver, XboxControllerButtonCode.X.getCode()).whenPressed(new DownShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.Y.getCode()).whenPressed(new UpShiftCommand());

     // setup buttons
     heightDownSelect = new JoystickButton(assistant, XboxControllerButtonCode.LB.getCode());
     heightUpSelect   = new JoystickButton(assistant, XboxControllerButtonCode.RB.getCode());
     captureRelease   = new JoystickButton(assistant, XboxControllerButtonCode.Y.getCode());
  }

  // Bind analog controls to functions to use by the commands
  // this way we only change it key/stick assignemnts once.

  // Use Triggers to directly make small adustments to the arm, raw stick units converted in
  // the CommandManager
  public double adjustHeight() {
    return Robot.m_oi.assistant.getTriggerAxis(Hand.kLeft) - Robot.m_oi.assistant.getTriggerAxis(Hand.kRight);
  }

  public double extensionInput() 
  {
    return Robot.m_oi.assistant.getY(Hand.kRight);
  }
  //assistant rotation input
  public double rotationInput() 
  {
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