/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package lib.controlsystem;

import lib.util.Util;

/**
 * PID Class for future use and testing
 * 
 * @param P is the proportionality constant. It's calculated by the error in the system at a moment in time - this is the best way to quickly eliminate error, but tends to overcompensate at higher values of kP (resulting in oscillation).
 * @param I is the integral constant. It's calculated by the total error of the system across a period of time - this results in changes in the steady state error (where oscillation has ceased).
 * @param D is the derivative constant. It's calculated by the rate of change of the error of the system - this removes transient error, resulting in less oscillation of error over time.
 */
public class PID {
    private double kP;
    private double kI;
    private double kD;

    private double min;
    private double max;

    private double LastErr;
    private double TotErr;


    public PID(double kP, double kI, double kD) {
        this(kP, kI, kD, Double.MIN_VALUE, Double.MAX_VALUE);
      }

 /**
   * @param kP  the propotionality constant
   * @param kI  the integral constant
   * @param kD  the derivitive constant
   * @param min minimum value of system
   * @param max maximum value of system
   */
  public PID(double kP, double kI, double kD, double min, double max) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.min = min;
    this.max = max;
  }

  public void reset(){
      LastErr = 0;
      TotErr = 0;
  }
  /**
   * Gets the value of the PID loop
   * Later, I need to make this capped because there might be errors in this in practice
   * @param current Input of Current Value
   * @param goal Expected Goal
   * @return Value after PID has been accounted for
   */
  public double getRawValue(double current, double goal){
    double p = goal - current;
    double i = TotErr;
    double d = p - LastErr;
    LastErr = p;
    TotErr += p;

    return kP*p + kI*i + kD*d;
  }
  public double getClampedValue(double current, double goal){
      double output = getRawValue(current, goal);
      output = Util.clamp(output, min, max);
      return output;

  }

  public double getMin() {
    return min;
  }

  public double getMax() {
    return max;
  }

  public void setP(double p) {
    this.kP = p;
  }

  public void setI(double i) {
    this.kI = i;
  }

  public void setD(double d) {
    this.kD = d;
  }

  public void setMax(double max){
      this.max = max;
  }
}