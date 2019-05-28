/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package lib.util;

/**
 * Literally just 254's DriveSignal class
 */
public class DriveSignal {

    private double left;
    private double right;

    /**
     * Delivers a drive signal with left and right inputs
     * @param left side of the drive
     * @param right side of the drive
     */
    public DriveSignal(double left, double right) {
        this.left = left;
        this.right = right;
    }
    
    public double getLeft() {
        return this.left;
    }

    public double getRight() {
        return this.right;
    }

    @Override
    public String toString() {
        return "Left: " + left + " Right: " + right;
    }

}
