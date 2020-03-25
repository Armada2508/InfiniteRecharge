package frc.lib.vision;

public class Resolution {
    private final int mX;
    private final int mY;

    /**
     * Create a new Resolution object
     * 
     * @param x The horizontal resolution
     * @param y The vertical resolution
     */
    public Resolution(int x, int y) {
        mX = x;
        mY = y;
    }

    /**
     * Get the horizontal resolution
     * @return The horizontal resolution
     */
    public int getX() {
        return mX;
    }

    /**
     * Get the vertical resolution
     * @return The vertical resolution
     */
    public int getY() {
        return mY;
    }
}