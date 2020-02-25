package frc.lib.vision;

public class FOV {
    public final double mX;
    public final double mY;

    /**
     * Create a new Resolution object
     * 
     * @param x The x resolution
     * @param y The y resolution
     */
    public FOV(double x, double y) {
        mX = x;
        mY = y;
    }

    /**
     * Get the x FOV
     * @return The x FOV
     */
    public double getX() {
        return mX;
    }

    /**
     * Get the y FOV
     * @return The y FOV
     */
    public double getY() {
        return mY;
    }
}