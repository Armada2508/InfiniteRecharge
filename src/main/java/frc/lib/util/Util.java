package frc.lib.util;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {}

    /**
     * @param value The value to check
     * @param maxMagnitude The max absolute value of the number
     * @return If the absolute value of a number is less than a maximum
     */
    public static boolean inRange(double value, double maxMagnitude) {
        return Math.abs(value) < maxMagnitude;
    }

    /**
     * 
     * @param value The value to check
     * @param min The minimum value
     * @param max The maximum value
     * @return If the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double value, double min, double max) {
        return value > min && value < max;
    }

    /**
     * Linearly interpolate between two values
     * @param a The first value
     * @param b The second value
     * @param x The amount to interpolate(0.0-1.0)
     * @return The interpolated value
     */
    public static double lerp(double a, double b, double x) {
        x = MathUtil.clamp(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    /**
     * Joins strings with a delimiter
     * @param delim The delimiter
     * @param strings The strings to combine
     * @return The final string
     */
    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    /**
     * Checks if the difference between two values is less than a margin of error(some small value)
     * @param a The first value
     * @param b The second value
     * @param epsilon The margin of error
     * @return If the difference between two values is less than {@code epsilon}
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
    /**
     * Checks if the difference between two values is less than a margin of error(some small value)
     * @param a The first value
     * @param b The second value
     * @return If the difference between two values is less than a small {@code epsilon} value
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }


    /**
     * Checks if the difference between two values is less than a margin of error(some small value)
     * @param a The first value
     * @param b The second value
     * @param epsilon The margin of error
     * @return If the difference between two values is less than {@code epsilon}
     */
    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /**
     * Checks if the difference between all values in a list is less than a margin of error
     * @param list The list of value
     * @param value The value to check with
     * @param epsilon The margin of error
     * @return If all of the values in the list are within {@code epsilon} of {@code value}
     */
    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    /**
     * @param angle An angle in radians
     * @return A colinear angle less than 2π and greater than or equal to 0 
     */
    public static double boundedAngle(double angle) {
        return angle % (2 * Math.PI);
    }

    /**
     * @param angle An angle in degrees
     * @return A colinear angle less than 360 and greater than or equal to 0 
     */
    public static double boundedAngleDegrees(double angle) {
        return angle % 360;
    }
    

    /**
     * @param rotation A Rotation2d object
     * @return A Rotation2d object less than 2π and greater than or equal to 0 
     */
    public static Rotation2d boundedAngle(Rotation2d rotation) {
        return new Rotation2d(boundedAngle(rotation.getRadians()));
    }
}