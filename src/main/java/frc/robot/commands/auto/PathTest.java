/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;


import frc.robot.Robot;

import lib.enums.TransmissionSide;
import lib.util.DriveSignal;

import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.command.Command;


/**
 * This is a command utilizing Jaci's pathfinder and her Pathweaver path creating utility (a tool inside of WPILib)
 * This is a direct implementation from the WPILib screensteps
 */
public class PathTest extends Command {
  private static final int k_ticks_per_rev = 1024; //TODO change this to match the actual encoders used
  private static final double k_wheel_diameter = 4.0 / 12.0; //Described in feet
  private static final double k_max_velocity = 10; //Described in feet/sec


  private static final String k_path_name = "Unnamed"; //path name as described in Path Weaver

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
  
  private Notifier m_follower_notifier;

  /**
   * This follows a designated path from the csv file created in the Path Weaver program - this can be extended to its specific subsystem file for logging purposes, but I just want to see if this works
   */
  private void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate(Robot.m_TestDrive.getPositionRaw(TransmissionSide.left));
      double right_speed = m_right_follower.calculate(Robot.m_TestDrive.getPositionRaw(TransmissionSide.right));
      double heading = Robot.m_TestDrive.getHeading();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;

      Robot.m_TestDrive.setSpeed(new DriveSignal(left_speed + turn, right_speed - turn));
    }
  }

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

    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder(Robot.m_TestDrive.getPositionRaw(TransmissionSide.left), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!TODO tune these values when I have a drivetrain
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    m_right_follower.configureEncoder(Robot.m_TestDrive.getPositionRaw(TransmissionSide.right), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line! TODO tune these values when I have a drivetrain
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

            
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
    
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

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
  }
}
