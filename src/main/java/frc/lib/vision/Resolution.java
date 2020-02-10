package frc.lib.vision;

public class Resolution {
    public final int m_x;
    public final int m_y;

    /**
     * Create a new Resolution object
     * 
     * @param x The x resolution
     * @param y The y resolution
     */
    public Resolution(int x, int y) {
        m_x = x;
        m_y = y;
    }

    /**
     * Get the x resolution
     * @return The x resolution
     */
    public int getX() {
        return m_x;
    }

    /**
     * Get the y resolution
     * @return The y resolution
     */
    public int getY() {
        return m_y;
    }
}