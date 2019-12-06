/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package lib.util;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.List;

public class Util {

    /**
     * Clamping ensures that a value doesn't fall above nor below a minimum or maximum value
     * @param value to be clamped in a specific range
     * @param min of the value being clamped
     * @param max of the value being clamped
     * @return a value between the minimum and maximum value
     */
    public static double clamp(double value, double min, double max) {
        if (value < min) {
            value = min;
        }
        if (value > max) {
            value = max;
        }
        return value;
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static double deadband(double v, double deadband) {
        return (Math.abs(v) < deadband) ? 0 : v;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double toFeet(double meters) {
        return meters * 3.28084;
    }

    public static double toMeters(double feet) {
        return feet * 0.3048;
    }

    public static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();
     
        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
