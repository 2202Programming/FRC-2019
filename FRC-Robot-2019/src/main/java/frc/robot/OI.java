/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.input.XboxControllerButtonCode;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.LimeLightArcadeDriveCommand;
import frc.robot.commands.arm.*;
//import frc.robot.commands.cargo.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.drive.shift.*;
import frc.robot.commands.intake.*;

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

  @SuppressWarnings({ "resource", })
  public OI() {
    // Drive Train Commands
    new JoystickButton(driver, XboxControllerButtonCode.A.getCode()).whenPressed(new DownShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.Y.getCode()).whenPressed(new UpShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.B.getCode()).whenPressed(new ToggleAutomaticGearShiftingCommand());
    new JoystickButton(driver, XboxControllerButtonCode.X.getCode()).whenPressed(new InvertDriveControlsCommand());
    new JoystickButton(driver, XboxControllerButtonCode.LB.getCode()).whileHeld(new TankDriveCommand());

    //Arm Commands
    new JoystickButton(assistant, XboxControllerButtonCode.Y.getCode()).whileHeld(new ExtendArmToPositionCommand(15));
    new JoystickButton(assistant, XboxControllerButtonCode.A.getCode()).whileHeld(new ExtendArmToPositionCommand(5));
    new JoystickButton(assistant, XboxControllerButtonCode.X.getCode()).whenPressed(new RotateArmToAngleCommand(90));
    new JoystickButton(assistant, XboxControllerButtonCode.B.getCode()).whenPressed(new RotateArmToAngleCommand(120));

    //Intake Commands
   // new JoystickButton(assistant, XboxControllerButtonCode.RB.getCode()).whenPressed(new OuttakeCommand());
    new JoystickButton(assistant, XboxControllerButtonCode.START.getCode()).whenPressed(new RotateWristCommand(15));
    new JoystickButton(assistant, XboxControllerButtonCode.BACK.getCode()).whenPressed(new RotateWristCommand(-15));

    //Driver assist commands (macros)
    
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
