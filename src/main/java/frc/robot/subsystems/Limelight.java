/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.logging.Logger;
import edu.wpi.first.networktables.*;
import frc.robot.commands.auto.*;

/**
 * Add your docs here.
 */
public class Limelight extends LoggableSubsystem {


  private static Limelight mInstance;
    public synchronized static Limelight getInstance() {
      if (mInstance == null) mInstance = new Limelight();
      return mInstance;
    }

      //x is the tx degree value from Limelight Network Table
      //y is the ty degree value from Limelight Network Table
      //area is the ta value from Limelight Network Table. The ratio of the object area to the entire picture area, as a percent.
      private double x, y, area;
  
      NetworkTable table; //TODO if null, change to the value under limelight()
      double[] data;
      double cameraHeight;
      double cameraAngle;
      double x_resolution = 320;
      double y_resolution = 240;
      double x_fov = 54;
      double y_fov = 41;
      double x_focal_length = x_resolution / (2*Math.tan(x_fov/2));
      double y_focal_length = y_resolution / (2*Math.tan(y_fov/2));
      double average_focal_length = (x_focal_length + y_focal_length) / 2;
    
      double distance, relativeAngle;
    
      public void setHeightAngle(double height, double angle){
        cameraHeight = height;
        cameraAngle = angle;
      }
      public double[] getData() {
        /** Whether the limelight has any valid targets (0 or 1) */
        NetworkTableEntry tv = table.getEntry("tv");
        /** Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) */
        NetworkTableEntry tx = table.getEntry("tx");
        /** Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
        NetworkTableEntry ty = table.getEntry("ty");
        /** Target Area (0% of image to 100% of image) */
        NetworkTableEntry ta = table.getEntry("ta");
        /** Skew or rotation (-90 degrees to 0 degrees) */
        NetworkTableEntry ts = table.getEntry("ts");
        /** The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency. */
        NetworkTableEntry tl = table.getEntry("tl");
    
        double[] data = new double[6];
    
        data[0] = tv.getDouble(0);
        data[1] = tx.getDouble(0);
        data[2] = ty.getDouble(0);
        data[3] = ta.getDouble(0);
        data[4] = ts.getDouble(0);
        data[5] = tl.getDouble(0);
        return data;
      }
        /** Turn on the Limelight LED */
    public void turnOnLED() {
      table.getEntry("ledMode").setNumber(0); 
    }
  
    /** Turn off the Limelight LED */
    public void turnOffLED() {
      table.getEntry("ledMode").setNumber(1); 
    }
      public Limelight()  //used to initalize the main, important things
      {
          table = NetworkTableInstance.getDefault().getTable("limelight");    //initializing the network table to grab values from limelight\
          setHeightAngle(6, 0);
          updateLimelight();
      }
  
      public void updateLimelight()
      {
          NetworkTableEntry tx = table.getEntry("tx"); //setting the N.T. entry to the tx value from limelight N.T.
          NetworkTableEntry ty = table.getEntry("ty"); //setting the N.T. entry to the ty value from limelight N.T.
          NetworkTableEntry ta = table.getEntry("ta"); //setting the N.T. entry to the ta value from limelight N.T.
          
      x = tx.getDouble(0.0);      //x is set to tx, and setting the default value to 0 if not recieving values from limelight
      y = ty.getDouble(0.0);      //y is set to ty, and setting the default value to 0 if not recieving values from limelight
      area = ta.getDouble(0.0);   //area is set to ta, and setting the default value to 0 if not recieving values from limelight
  
      SmartDashboard.putNumber("LimelightX", x);          //adding the values to SmartDashboard
      SmartDashboard.putNumber("LimelightY", y);          //adding the values to SmartDashboard
      SmartDashboard.putNumber("LimelightArea", area);    //adding the values to SmartDashboard
      }
  
      public double getX()            //get x, because x is private
      {
          return this.x;
      }
  
      public double getY()            //get y, because y is private
      {
          return this.y;
      }
  
      public double getArea()         //get area, because area is private
      {
          return this.area;
      }
        /**
     * Get the area of the tracked target as a percentage from 0% to 100%
     * @return area as percentage of total area 
     */
      public double getTargetArea() {
          return (table.getEntry("ta")).getDouble(0);
      }
      /**
     * Get the dx from crosshair to tracked target
     * @return skew from -90 to 0 degrees
     */
    public double getTargetSkew() {
      return (table.getEntry("ts")).getDouble(0);
    }
    
    /**
     * Get the latency from camera data to NT entry
     * @return pipeline latency contribution
     */
    public double getPipelineLatency() {
      return (table.getEntry("tl")).getDouble(0);
    }
  
    /**
     * Get the dx from crosshair to tracked target
     * @return dx
     */
    public double getDx() {
      return (table.getEntry("tx")).getDouble(0);
    }
  
    /**
     * Get the dy from crosshair to tracked target
     */
    public double getDy() {
      return (table.getEntry("ty")).getDouble(0);
    }
  
    /**
     * Return the number of targets currently being tracked
     * @return currently tracked targets
     */
    public double getTrackedTargets() {
      return (table.getEntry("tv")).getDouble(0);
    }
  
    /**
     * Get the current delta x (left/right) angle from crosshair to vision target
     * @return delta x in degrees to target
     */
    public double getDxAngle() {
      return Math.toDegrees(
        Math.atan(
          getDx() / average_focal_length
      ));
    }
  
    /**
     * Get the current elevation (delta y) angle from crosshair to vision target
     * @return degrees of elevation from crosshair to target 
     */
    public double getDyAngle() {
      return Math.toDegrees(
        Math.atan(
          getDy() / average_focal_length
      ));
    }
  
    /**
     * Get the distance (in the same units as elevation) from a tracked vision target
     * @param targetElevation
     * @return distance to target
     */
    public double getDistanceToFixedPoint(double targetElevation){
      return (targetElevation - cameraHeight) / Math.tan(cameraAngle + (table.getEntry("ty").getDouble(0)));
    }
    public void logPeriodicIO(){
      Logger.log(this.getClass().getSimpleName(), true);
      Logger.log("Vision Targets Tracked", getTrackedTargets());
      Logger.log("Camera Height", cameraHeight);
      Logger.log("Camera Angle", cameraAngle);
      Logger.log("Limelight Data", getData());
      if(getTrackedTargets() != 0){
        Logger.log("Limelight Change in X Angle", getDxAngle());
        Logger.log("Limelight Change in Y Angle", getDyAngle());

      }

    }
    public void initDefaultCommand() {
      //setDefaultCommand(new VisionTest(0.1, 0.5, 100));
    }
}
