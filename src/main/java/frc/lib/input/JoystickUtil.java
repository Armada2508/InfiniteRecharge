package frc.lib.input;

public class JoystickUtil {

    /**
     * Applies a deadband function to the input
     * 
     * @param value The input value
     * @param threshold The deadband threshold
     * @return The deadband of the input at the set threshold
     */
    public static double deadband(double value, double threshold) {
        if(Math.abs(value) > threshold) {
            return value;
        } else {
            return 0.0;
        }
    }
}