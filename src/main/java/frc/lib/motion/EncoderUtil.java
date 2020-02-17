package frc.lib.motion;

public class EncoderUtil {
    /**
     * Converts from encoder units to real-world units
     * @param sensorPosition The current value read from the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return Distance traveled
     */
    public static double toDistance(int sensorPosition, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        //               Encoder Units             Convert to revolutions      Convert to distance
        return ((double)sensorPosition / (double)(encoderUnitsPerRev * gearRatio)) * Math.PI * wheelDiameter;
    }
    
    /**
     * Converts from real-world units to encoder units
     * @param distance The distance traveled
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return Distance in encoder units
     */
    public static double fromDistance(double distance, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        //               Encoder Units             Convert to revolutions      Convert to distance
        return distance / ( Math.PI * wheelDiameter ) * (double)encoderUnitsPerRev * gearRatio;
    }



    /**
     * Converts encoder units per second to units per second
     * @param velocity The curent velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return
     */

    public static double toVelocity(int velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        return toVelocity(velocity, encoderUnitsPerRev, gearRatio, wheelDiameter, 1.0);
    }
    
    /**
     * Converts encoder units per time unit specified to velocity
     * @param velocity The curent velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @param time The time period over which this velocity was measured(e.g. if velocity is measured in units/100ms the time value would be 0.1)
     * @return
     */
    public static double toVelocity(int velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter, double time) {
        return toDistance(velocity, encoderUnitsPerRev, gearRatio, wheelDiameter) / time;
    }


    /**
     *  Converts from units per second to encoder units per second
     * @param velocity The curent velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return
     */

    public static double fromVelocity(double velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        return fromVelocity(velocity, encoderUnitsPerRev, gearRatio, wheelDiameter, 1.0);
    }
    
    /**
     * Converts from units per second specified to encoder units per time unit specified 
     * @param velocity The curent velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @param time The time period over which this velocity was measured(e.g. if velocity is measured in units/100ms the time value would be 0.1)
     * @return
     */
    public static double fromVelocity(double velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter, double time) {
        return fromDistance(velocity, encoderUnitsPerRev, gearRatio, wheelDiameter) * time;
    }


    /**
     * Converts encoder velocity to RPM
     * @param velocity The encoder velocity
     * @param encoderUnitsPerRev The number of encoder units per revolution
     * @param gearRatio The gear ratio between the output and the motor
     * @param time The time over which the velocity was measued(e.g. 100ms for Talons)
     * @return RPM of the output
     */
    public static double toRPM(double velocity, int encoderUnitsPerRev, double gearRatio, double time) {
        return (velocity * 60.0 / (double)encoderUnitsPerRev) / (gearRatio * time);
    }

    

    /**
     * Converts RPM to encoder velocity
     * @param velocity The input RPM
     * @param encoderUnitsPerRev The number of encoder units per revolution
     * @param gearRatio The gear ratio between the output and the motor
     * @param time The time over which the velocity is measued(e.g. 100ms for Talons)
     * @return RPM encoder velocity
     */
    public static double fromRPM(double RPM, int encoderUnitsPerRev, double gearRatio, double time) {
        return RPM * ((double)encoderUnitsPerRev / 60.0) * time * gearRatio;
    }

}