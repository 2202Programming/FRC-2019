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
import frc.robot.commands.cargo.tests.IntakeTestCmd;
import frc.robot.commands.cargo.tests.OuttakeTestCmd;
import frc.robot.commands.climb.tests.CharonSolenoidTestCmd;
import frc.robot.commands.climb.tests.ClimbMotorTestCmd;
import frc.robot.commands.climb.tests.PawlSolenoidTestCmd;
import frc.robot.commands.climb.tests.RollerMotorTestCmd;
import frc.robot.commands.drive.*;
import frc.robot.commands.drive.shift.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.intake.tests.IntakeTestCommand;
import frc.robot.commands.intake.tests.SolenoidTestCommand;
import frc.robot.commands.intake.tests.VacuumTestCommand;

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
  public JoystickButton huntSelect;         // used in hunting modes
  public JoystickButton heightSelect;       // used in delivery modes
  public JoystickButton captureRelease;     // used in delivery modes to go back to hunting

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
    huntSelect     = new JoystickButton(assistant, XboxControllerButtonCode.LB.getCode());
    heightSelect   = new JoystickButton(assistant, XboxControllerButtonCode.RB.getCode());
    captureRelease = new JoystickButton(assistant, XboxControllerButtonCode.A.getCode());

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

    //gearbox tests
    new JoystickButton(driver, XboxControllerButtonCode.A.getCode()).whenPressed(new DownShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.Y.getCode()).whenPressed(new UpShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.B.getCode()).whileHeld(new IntakeTestCmd(0.3));
    new JoystickButton(driver, XboxControllerButtonCode.X.getCode()).whileHeld(new OuttakeTestCmd(0.3));

    //Climber tests
    new JoystickButton(switchBoard, 7).whenPressed(new PawlSolenoidTestCmd(true));
    new JoystickButton(switchBoard, 8).whileActive(new ClimbMotorTestCmd(0.3));
    new JoystickButton(switchBoard, 9).whenPressed(new CharonSolenoidTestCmd(true));
    new JoystickButton(switchBoard, 10).whileActive(new RollerMotorTestCmd(0.5));
    new JoystickButton(switchBoard, 11).whileActive(new ClimbMotorTestCmd(-0.3));


     // setup buttons
     huntSelect     = new JoystickButton(assistant, XboxControllerButtonCode.LB.getCode());
     heightSelect   = new JoystickButton(assistant, XboxControllerButtonCode.RB.getCode());
     captureRelease = new JoystickButton(assistant, XboxControllerButtonCode.Y.getCode());
  }

  // Bind analog controls to functions to use by the commands
  // this way we only change it key/stick assignemnts once.
  public double adjustHeightDown() {
    return Robot.m_oi.assistant.getTriggerAxis(Hand.kLeft);
  }

  public double adjustHeightUp() {
    return Robot.m_oi.assistant.getTriggerAxis(Hand.kRight);
  }

  public double extensionInput() 
  {
    return Robot.m_oi.assistant.getY(Hand.kRight);
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