/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package lib.geometry;

/**
 * Add your docs here.
 */
public class Position2d extends Point {
    private double angle;
    private double lastAngle;

    public Position2d(double angle){
        this.angle = angle;
    }
    
    public Position2d(final Point p, double angle){
        super(p.getX(), p.getY());
        this.angle = angle;
    }

    public Position2d(double x, double y, double angle){
        super(x, y);
        this.angle = angle;
    }

    public double getAngle(){
        return this.angle;
    }
    public double rotate1(double distance, double initialAngle){
        Point rotate1 = this.translate(distance, initialAngle);
        return Math.atan2(rotate1.getY(), rotate1.getX()) - initialAngle;
    }
    public double rotate2(double angle, double distance){
        lastAngle = angle;
        return angle - lastAngle - rotate1(distance, angle);
    }

    @Override
    public Position2d clone(){
        return new Position2d(getX(), getY(), angle);
    }

    @Override
    public String toString(){
        return "X: " + getX() + " Y:" + getY() + " Angle: " + getAngle();
    }
}
