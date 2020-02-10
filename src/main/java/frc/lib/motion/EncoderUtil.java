package frc.lib.motion;

public class EncoderUtil {
    /**
     * 
     * @param sensorPosition The current value read from the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return
     */
    public static double toDistance(int sensorPosition, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        //               Encoder Units             Convert to revolutions      Convert to distance
        return ((double)sensorPosition / (double)(encoderUnitsPerRev * gearRatio)) * Math.PI * wheelDiameter;
    }

    /**
     * 
     * @param velocity The curent velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return
     */

    public static double toVelocity(double velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        return toVelocity(velocity, encoderUnitsPerRev, gearRatio, wheelDiameter, 1.0);
    }
    
    /**
     * 
     * @param velocity The curent velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @param time The time period over which this velocity was measured(e.g. if velocity is measured in units/100ms the time value would be 0.1)
     * @return
     */
    public static double toVelocity(double velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter, double time) {
        //            Encoder Units          Convert to revolutions             Convert to distance  Convert to seconds
        return (((double)velocity / (double)(encoderUnitsPerRev * gearRatio)) * Math.PI * wheelDiameter) / time;
    }

}