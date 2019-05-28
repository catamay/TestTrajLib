/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package lib.controlsystem;

import lib.util.Util;

/**
 * PIDF Controller, PID is defined in {@link PID}
 * @param Feedforward represents a value multiplied by a goal to reduce steady state error
 */
public class PIDF extends PID {
    private double kF;

    public PIDF(double kP, double kI, double kD, double kF) {
      super(kP, kI, kD);
      this.kF = kF;
    }
  
    public PIDF(double kP, double kI, double kD, double kF, double min, double max) {
      super(kP, kI, kD, min, max);
      this.kF = kF;
    }
    @Override
    public double getRawValue(double cur, double goal) {
        double out = super.getRawValue(cur, goal);
        out += goal * kF;
        return out;
    }
  
    @Override
    public double getClampedValue(double cur, double goal) {
      double out = getClampedValue(cur, goal);
      out = Util.clamp(out, getMin(), getMax());
      return out;
    }
}
