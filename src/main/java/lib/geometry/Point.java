/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package lib.geometry;

/**
 * Describes a point on a 2-dimensional plane of X and Y - can be useful for robot localization for more complex implementations of code
 */
public class Point {
    private double x, y;

    public Point(){}

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Point(final Point p){
        this(p.x, p.y);
    }

    public Point translate(double distance, double angle){
        double deltax = distance * Math.cos(angle);
        double deltay = distance * Math.sin(angle);
        return new Point(x + deltax, y + deltay);
    }

    public double getX(){
        return this.x;
    }

    public double getY(){
        return this.y;
    }


    public Point clone(){
        return new Point(x,y);
    }
}
