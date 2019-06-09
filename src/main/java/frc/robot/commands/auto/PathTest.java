/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;


import frc.robot.Robot;
import frc.robot.subsystems.Pathing;

import java.io.IOException;

import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import edu.wpi.first.wpilibj.command.Command;


/**
 * This is a command utilizing Jaci's pathfinder and her Pathweaver path creating utility (a tool inside of WPILib)
 * This is a direct implementation from the WPILib screensteps
 */
public class PathTest extends Command {
  
  private static final int k_ticks_per_rev = 42; //TODO change this to match the actual encoders used
  private static final double k_wheel_diameter = 4.0 / 12.0; //Described in feet
  private static final double k_max_velocity = 10; //Described in feet/sec
  private static final String k_path_name = "Unnamed"; //path name as described in Path Weaver
  private Trajectory left_trajectory;{
    try{
      left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
    }catch(IOException e){
      e.printStackTrace();
    }
  }
  private Trajectory right_trajectory;{
    try{
      left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
    }catch(IOException e){
      e.printStackTrace();
    }
  }

  public PathTest() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_TestDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Pathing.getInstance().init(left_trajectory, right_trajectory, k_ticks_per_rev, 1.0, 1.0, 0, 0, 0, 0, k_max_velocity,  k_wheel_diameter);


    
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Pathing.getInstance().execute(left_trajectory);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Pathing.getInstance().end();
  }
}
