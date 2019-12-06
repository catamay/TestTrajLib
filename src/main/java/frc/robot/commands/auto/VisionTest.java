/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Limelight;
import lib.util.*;
import lib.logging.Logger;
/**
 * An example command.  You can replace me with your own command.
 */
public class VisionTest extends Command {
  double timeout,
         targetSpeed,
         targetPercentOfFrame,
         angleDeltaX,
         angleDeltaY,
         forwardSpeed,
         turnSpeed;

  boolean followRange = false;

  double targetSizeSetpoint = 6;

  boolean noCurrentTarget = false;

  boolean tooClose = false;

  boolean hadTarget = false;
  double lastKnownYaw;
  Limelight m_Limelight;

  /**
   * Vision Test Command (The speed seems wrong because it's going off of percent output atm, but that can be fixed relatively easily)
   * @param speed of the Robot Drivetrain
   * @param timeout is the time before the action is timed out (in ms)
   */
  public VisionTest(double speed, double timeout) {
      this.timeout = timeout;
      this.targetSpeed = speed;
      requires(Robot.m_TestDrive);
  }
  /**
   * Vision Test Command (The speed seems wrong because it's going off of percent output atm, but that can be fixed relatively easily)
   * @param speed of the Robot Drivetrain
   * @param targetPercentOfFrame is the intended percent of the target area to be seen by the Limelight
   * @param timeout is the time before the action is timed out (in ms)
   */
  public VisionTest(double speed, double targetPercentOfFrame, double timeout) {
    this.timeout = timeout;
    this.targetSpeed = speed;
    this.targetPercentOfFrame = targetPercentOfFrame;
    this.followRange = true;
    requires(Robot.m_TestDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(timeout);
    m_Limelight = Limelight.getInstance();
    angleDeltaX = m_Limelight.getDx();
    m_Limelight.setHeightAngle(3.5, 25);
    //targetPercentOfFrame = m_Limelight.getTargetArea();


  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (m_Limelight.getData()[0] != 0) {
      noCurrentTarget = false;
      hadTarget = true;

      double limelightData = m_Limelight.getX();
      double sizeData = m_Limelight.getArea();

      lastKnownYaw = limelightData;
      turnSpeed = Util.limit( limelightData * (1.0/27), -targetSpeed, targetSpeed);

      Logger.log("Turn speed:", turnSpeed);

      forwardSpeed = 0;

      double distanceRatio = targetSizeSetpoint - sizeData;
      
      double forwardSpeed = distanceRatio * 0.2;

      if (sizeData > targetSizeSetpoint) { 
        tooClose = true; 
        Logger.log("The Robot is", "Too close"); 
      }

      
      if(distanceRatio != distanceRatio + 0.001){
        forwardSpeed = 10 * (distanceRatio) / 100;
        if ( forwardSpeed > 0.5 ) { forwardSpeed = 0.5;}
        if ( forwardSpeed < -0.5 ) { forwardSpeed = -0.5;}
        SmartDashboard.putNumber("forward speed", forwardSpeed);
        SmartDashboard.putNumber("Turn speed", turnSpeed);
      }


      
      
     
      Robot.m_TestDrive.setSpeed(new DriveSignal(forwardSpeed + turnSpeed, forwardSpeed - turnSpeed));
    } else {
      noCurrentTarget = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double datainYaw = m_Limelight.getData()[1];
    double datainSize = m_Limelight.getData()[3];
    


    return (isTimedOut() || (hadTarget && noCurrentTarget) || (((datainSize > targetSizeSetpoint))  && (datainYaw < 0.5) ));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.log("Finished Vision Test", isFinished());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}