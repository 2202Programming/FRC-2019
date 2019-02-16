/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *  Extends the WPILIB Subsystem
 *  Require a few additional methods for our subsystems
 */
  public abstract class ExtendedSubSystem extends Subsystem {

    ExtendedSubSystem(String name) {
      super(name);
    }


  // require subsystems to provide a zero command
  public abstract Command zeroSubsystem();

  // ensure every subsystem has a log call
  public void log() { }

}
