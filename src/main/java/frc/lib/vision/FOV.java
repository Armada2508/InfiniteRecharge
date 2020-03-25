package frc.lib.vision;

public class FOV {
    public final double mX;
    public final double mY;

    /**
     * Create a new Field-of-View object
     * 
     * @param x The horizontal field of vew
     * @param y The vertical field of view
     */
    public FOV(double x, double y) {
        mX = x;
        mY = y;
    }

    /**
     * Get the horizontal FOV
     * @return The horizontal FOV
     */
    public double getX() {
        return mX;
    }

    /**
     * Get the vertical FOV
     * @return The vertical FOV
     */
    public double getY() {
        return mY;
    }
}