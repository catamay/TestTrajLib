/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.UseDrive;

import lib.util.*;
import lib.enums.TransmissionSide;
import lib.logging.Logger;


import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.*;         //This is commented out because it's unused right now.


/**
 * Test Drivetrain using any number of Rev Neo motors (typically 2 or 3 per drive train)
 * This Drivetrain assumes Tank Drive and no shifting gearbox is present (however, we can get the latter worked out pretty easily, the former requires ... *Math*)
 */
public class TestDrive extends LoggableSubsystem{
  private static TestDrive mInstance;
  private CANSparkMax leftMaster, rghtMaster;
  private CANSparkMax leftSlaveA, leftSlaveB, rghtSlaveA, rghtSlaveB; //Visual Studio Code shows these as unused because they are used solely in the private method TestDrive().
  private DriveSignal lastSignal;
  private PigeonIMU mPidgey;
  private double leftOffset, rightOffset;


  public synchronized static TestDrive getInstance() {
    if (mInstance == null) {
        mInstance = new TestDrive();
    }
    return mInstance;
}
  private TestDrive(){
    leftMaster = MotorFactory.createBrushlessNeo(Constants.TestDrive.kLeftMasterID);
    rghtMaster = MotorFactory.createBrushlessNeo(Constants.TestDrive.kRghtMasterID);
    leftSlaveA = MotorFactory.createSlavedSpark(Constants.TestDrive.kLeftSlaveAID, leftMaster);
    leftSlaveB = MotorFactory.createSlavedSpark(Constants.TestDrive.kLeftSlaveBID, leftMaster);
    rghtSlaveA = MotorFactory.createSlavedSpark(Constants.TestDrive.kRghtSlaveAID, rghtMaster);
    rghtSlaveB = MotorFactory.createSlavedSpark(Constants.TestDrive.kRghtSlaveBID, rghtMaster);

    mPidgey = new PigeonIMU(Constants.TestDrive.kLeftSlaveAID);
     
    leftMaster.setInverted(true);
    leftSlaveA.setInverted(true);
    leftSlaveB.setInverted(true);
     
  }

  //These methods are made with the CTRE Pigeon IMU in mind
  public double getHeading(){
    double[] ypr = new double[3];
    mPidgey.getYawPitchRoll(ypr);
    return ypr[0];
  }
  /**
   * Set the heading of the Pigeon Gyroscope
   * @param deg is angle you wish to set the gyro to
   */
  public void setHeading(double deg){
      mPidgey.setYaw(deg);
  }
  

  //These methods are made with the CAN Spark Max in mind and can easily be adjusted to work with a CANSpark MAX.

  public void setPosConversionFactor(double factor){
      leftMaster.getEncoder().setPositionConversionFactor(factor);
      rghtMaster.getEncoder().setPositionConversionFactor(factor);
  }
  /**
   * Gets the Raw Sensor Position of a Neo Drivetrain *USE ONLY WHEN YOU ARE USING RAW SENSOR UNITS AS THE POSITION CONVERSION FACTOR*
   * @param side of the drivetrain transmission
   * @return Position (in raw sensor units) relative to the distance traveled by the robot (positive is forward, negative is backward)
   */
    public int getPositionRaw(TransmissionSide side){
        switch(side){  
            case left:
                  return (int)Math.round(leftMaster.getEncoder().getPosition()- leftOffset);
            case right:
                  return (int)Math.round(rghtMaster.getEncoder().getPosition()- rightOffset);
            default:
                  return (int)Math.round(leftMaster.getEncoder().getPosition()- leftOffset);
                }
    }
    /**
     * Gets the Motor Speed of a Neo Drivetrain
     * @param side of the drivetrain transmission
     * @return Velocity (in motor RPM) of the robot
     */
    public double getSpeedRaw(TransmissionSide side){
        switch(side){  
            case left:
                  return leftMaster.getEncoder().getVelocity();
            case right:
                  return rghtMaster.getEncoder().getVelocity();
            default:
                  return leftMaster.getEncoder().getVelocity();
                }
    }

    /**
     * 
     * @param side of the drivetrain transmission
     * @return Distance travelled by the robot's transmission side in feet 
     */
    public double getDistTravelled(TransmissionSide side){
        return getPositionRaw(side)*Math.PI*Constants.TestDrive.WHEEL_DIAMETER/Constants.TestDrive.ENCODER_TO_WHEEL_RATIO;
    }

  /**
   * Sets the speed of the drivetrain (in percent) - I'm working out how this will translate to velocity based control methods
   * @param signal
   */
  public void setSpeed(DriveSignal signal){
    lastSignal = signal;
    leftMaster.set(signal.getLeft());
    rghtMaster.set(signal.getRight());
  }

  /**
   * This gives the last Drive Signal to allow logging of the drivetrain's constants motion over time and what values it receives from its inputs/outputs
   * @return lastSignal - the last Drive Signal delivered by IO
   */
  public DriveSignal getLast(){
    if(lastSignal == null){
      lastSignal = new DriveSignal(0, 0);
      return lastSignal;
    }else{
    return lastSignal;
    }
  }
  
  /**
   * Curvature Drive is the drive method used by Team 254, where turning is determined by the curvature of the robot, or the amount that the robot has turned
   * @param linearPercent is the percent output of the robot's forward (or backward) motion
   * @param curvaturePercent is the percent output of the robot's turning motion, or in other words, the relative amount the robot is turning
   * @param isQuickTurn is whether or not the robot is turning faster or slower than normal (determined by variables inside constants)
   */
  public void curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn) {
    double quickStopAccumulator = 0.0;
    double angularPower;
    boolean overPower;
    linearPercent = Util.limit(linearPercent,1);
    linearPercent = Util.deadband(linearPercent, 0.1);

    curvaturePercent = Util.deadband(curvaturePercent, 0.1);

    if (isQuickTurn) {
        if (Math.abs(linearPercent) < Constants.TestDrive.kQuickStopThreshold) {
            quickStopAccumulator = (1 - Constants.TestDrive.kQuickStopAlpha) * quickStopAccumulator +
            Constants.TestDrive.kQuickStopAlpha * curvaturePercent * 2.0;
        }

        overPower = true;
        angularPower = curvaturePercent;
    } else {
        overPower = false;
        angularPower = Math.abs(linearPercent) * curvaturePercent - quickStopAccumulator;

        if(quickStopAccumulator > 1){
            quickStopAccumulator -= 1;
        }else if(quickStopAccumulator < -1){
            quickStopAccumulator += 1;
        }else{
            quickStopAccumulator = 0;
        }
    }

    double leftMotorOutput = linearPercent + angularPower;
    double rightMotorOutput = linearPercent - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
        if(leftMotorOutput > 1.0){
            rightMotorOutput -= leftMotorOutput - 1.0;
            leftMotorOutput = 1.0;
        }else if(rightMotorOutput > 1.0) {
            leftMotorOutput -= rightMotorOutput - 1.0;
            rightMotorOutput = 1.0;
        }else if(leftMotorOutput < -1.0) {
            rightMotorOutput -= leftMotorOutput + 1.0;
            leftMotorOutput = -1.0;
        }else if(rightMotorOutput < -1.0){
            leftMotorOutput -= rightMotorOutput + 1.0;
            rightMotorOutput = -1.0;
        }
    }

    // Normalize the wheel speeds (keep them between -1 and 1)
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
        leftMotorOutput /= maxMagnitude;
        rightMotorOutput /= maxMagnitude;
    }
    
    setSpeed(new DriveSignal(leftMotorOutput,rightMotorOutput));
}

public void zeroSensors() {
    leftOffset = getPositionRaw(TransmissionSide.left); //leftMaster.getEncoder().getPosition(); 
    rightOffset = getPositionRaw(TransmissionSide.right); //rghtMaster.getEncoder().getPosition();
    setHeading(0);
}

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new UseDrive());
  }

  @Override
  public void logPeriodicIO(){
    Logger.log(this.getClass().getSimpleName(), true);
    Logger.log("Left Sensor Position", getPositionRaw(TransmissionSide.left));
    Logger.log("Total left side displacement", getDistTravelled(TransmissionSide.left));
    Logger.log("Left Sensor Velocity", getSpeedRaw(TransmissionSide.left));
    Logger.log("Right Sensor Position", getPositionRaw(TransmissionSide.right));
    Logger.log("Right Sensor Velocity", getSpeedRaw(TransmissionSide.right));
    Logger.log("Total right side displacement", getDistTravelled(TransmissionSide.right));

    Logger.log("Last Drivesignal", getLast());
  }

}