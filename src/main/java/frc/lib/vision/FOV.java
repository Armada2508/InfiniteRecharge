package frc.lib.vision;

public class FOV {
    public final double m_x;
    public final double m_y;

    /**
     * Create a new Resolution object
     * 
     * @param x The x resolution
     * @param y The y resolution
     */
    public FOV(double x, double y) {
        m_x = x;
        m_y = y;
    }

    /**
     * Get the x FOV
     * @return The x FOV
     */
    public double getX() {
        return m_x;
    }

    /**
     * Get the y FOV
     * @return The y FOV
     */
    public double getY() {
        return m_y;
    }
}