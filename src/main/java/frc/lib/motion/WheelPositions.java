package frc.lib.motion;

public class WheelPositions {
    private double m_RWheelPos;
    private double m_LWheelPos;

    /**
     * An object that stores two wheel positions
     * @param rWheelPos The right wheel position
     * @param lWheelPos The left wheel position
     */
    public WheelPositions(double rWheelPos, double lWheelPos) {
        m_RWheelPos = rWheelPos;
        m_LWheelPos = lWheelPos;
    }

    /**
     * An object that stores two wheel positions.<br>
     * <br>
     * This object will be initialized with both wheel positions at 0.0
     */
    public WheelPositions() {
        this(0.0, 0.0);
    }

    /**
     * @return The right wheel position
     */

    public double getRight() {
        return m_RWheelPos;
    }

    /**
     * @return The left wheel position
     */

    public double getLeft() {
        return m_LWheelPos;
    }

    /**
     * Prints both wheel positions
     */
    @Override
    public String toString() {
        return ("Right Wheel Position: " + m_RWheelPos + ", Left Wheel Position: w" + m_LWheelPos);
    }
}