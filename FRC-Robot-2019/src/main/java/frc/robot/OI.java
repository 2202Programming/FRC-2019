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
import frc.robot.commands.*;

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
  private XboxController xboxController0 = new XboxController(0);
  private XboxController xboxController1 = new XboxController(1);
  private XboxController switchBoard = new XboxController(2);

  public OI() {
    //Drive Train Commands
    new JoystickButton(xboxController0, XboxControllerButtonCode.A.getCode()).whenPressed(new DownShiftCommand());
    new JoystickButton(xboxController0, XboxControllerButtonCode.Y.getCode()).whenPressed(new UpShiftCommand());
    new JoystickButton(xboxController0, XboxControllerButtonCode.B.getCode()).whenPressed(new ToggleAutomaticGearShiftingCommand());
    new JoystickButton(xboxController0, XboxControllerButtonCode.X.getCode()).whenPressed(new InvertDriveControlsCommand());
    new JoystickButton(xboxController0, XboxControllerButtonCode.LB.getCode()).whileHeld(new TankDriveCommand());
    
    //Arm Commands
    new JoystickButton(xboxController1, XboxControllerButtonCode.Y.getCode()).whileHeld(new ExtendArmCommand());
    new JoystickButton(xboxController1, XboxControllerButtonCode.A.getCode()).whileHeld(new RetractArmCommand());
    new JoystickButton(xboxController1, XboxControllerButtonCode.X.getCode()).whileHeld(new RotateArmForwardCommand());
    new JoystickButton(xboxController1, XboxControllerButtonCode.B.getCode()).whileHeld(new RotateArmForwardCommand());
    new JoystickButton(xboxController1, XboxControllerButtonCode.LB.getCode()).whenPressed(new IntakeCommand());
    new JoystickButton(xboxController1, XboxControllerButtonCode.RB.getCode()).whenPressed(new OuttakeCommand());
  }

  public XboxController getController0() {
    return xboxController0;
  }

  public XboxController getController1() {
    return xboxController1;
  }

  public XboxController getSwitchBoard() {
    return switchBoard;
  }

}
