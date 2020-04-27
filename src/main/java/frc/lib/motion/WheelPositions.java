package frc.lib.motion;

public class WheelPositions {
    private double mRWheelPos;
    private double mLWheelPos;

    /**
     * An object that stores two wheel positions
     * @param rWheelPos The right wheel position
     * @param lWheelPos The left wheel position
     */
    public WheelPositions(double rWheelPos, double lWheelPos) {
        mRWheelPos = rWheelPos;
        mLWheelPos = lWheelPos;
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
        return mRWheelPos;
    }

    /**
     * @return The left wheel position
     */

    public double getLeft() {
        return mLWheelPos;
    }

    /**
     * Prints both wheel positions
     */
    @Override
    public String toString() {
        return ("Left Wheel Position: " + mLWheelPos + ", Right Wheel Position: w" + mRWheelPos);
    }
}