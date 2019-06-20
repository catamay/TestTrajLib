/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;

import lib.enums.TransmissionSide;
import lib.util.DriveSignal;


import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;


/**
 * Add your docs here.
 */
public class Pathing {


  private static Pathing mInstance;

  public synchronized static Pathing getInstance() {
    if (mInstance == null) {
        mInstance = new Pathing();
    }
    return mInstance;
}

private Pathing(){}

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
  
  private Notifier m_follower_notifier;

  /**
   * This follows a designated path from the csv file created in the Path Weaver program - this can be extended to its specific subsystem file for logging purposes, but I just want to see if this works
   */
  public void followPath() {
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

  public void init(Trajectory Left, Trajectory Right, int Ticks, double leftkP, double rghtkP, double leftkI, double rghtkI, double leftkD, double rghtkD, double maxVel, double wheelDiam){
    
    m_left_follower = new EncoderFollower(Left);
    m_right_follower = new EncoderFollower(Right);

    TestDrive.getInstance().setPosConversionFactor(Ticks);
    
    m_left_follower.configureEncoder(Robot.m_TestDrive.getPositionRaw(TransmissionSide.left), Ticks, wheelDiam);
    // You must tune the PID values on the following line!TODO tune these values when I have a drivetrain
    m_left_follower.configurePIDVA(leftkP, leftkI, leftkD, 1 / maxVel, 0);

    m_right_follower.configureEncoder(Robot.m_TestDrive.getPositionRaw(TransmissionSide.right), Ticks, wheelDiam);
    // You must tune the PID values on the following line! TODO tune these values when I have a drivetrain
    m_right_follower.configurePIDVA(rghtkP, rghtkI, rghtkD, 1 / maxVel, 0);

            
    m_follower_notifier = new Notifier(this::followPath);
  }

  public void execute(Trajectory Left){
    m_follower_notifier.startPeriodic(Left.get(0).dt);
  }
  public void end(){
    m_follower_notifier.stop();
    Robot.m_TestDrive.setSpeed(new DriveSignal(0, 0));
  }
}
