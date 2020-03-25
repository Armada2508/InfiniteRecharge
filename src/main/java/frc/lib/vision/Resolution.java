package frc.lib.vision;

public class Resolution {
    private final int mX;
    private final int mY;

    /**
     * Create a new Resolution object
     * 
     * @param x The x resolution
     * @param y The y resolution
     */
    public Resolution(int x, int y) {
        mX = x;
        mY = y;
    }

    /**
     * Get the x resolution
     * @return The x resolution
     */
    public int getX() {
        return mX;
    }

    /**
     * Get the y resolution
     * @return The y resolution
     */
    public int getY() {
        return mY;
    }
}