/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import lib.logging.LogManager;
import lib.logging.LoggableClass;

/**
 * Wrapper to allow for a WPI CommandBased subsystem to be logged with the information needed, thus implying the use of only one line of code to define the Robot logs.
 */
public abstract class LoggableSubsystem extends Subsystem implements LoggableClass {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public LoggableSubsystem(){
    LogManager.getInstance().addLoggable(this);
  }

  public abstract void logPeriodicIO();

}
