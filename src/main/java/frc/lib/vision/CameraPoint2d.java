package frc.lib.vision;

public class CameraPoint2d {
    private double mX;
    private double mY;
    private boolean mIsAngle;
    private FOV mFov;
    private Resolution mRes;

    /**
     * Creates a new CameraPoint2d
     * 
     * @param tx The x angle of the target relative to the camera
     * @param ty The y angle of the target relative to the camera
     */
    public CameraPoint2d(double tx, double ty) {
        this(tx, ty, true);
    }
    
    /**
     * Creates a new CameraPoint2d
     * 
     * @param x The x coordinate
     * @param y The y coordinate
     * @param angle If these CameraPoint2d values are angles
     */
    public CameraPoint2d(double x, double y, boolean angle) {
        if(angle) {
            mIsAngle = angle;
            mX = x;
            mY = y;
        }
    }

    /**
     * Get the x coordinate
     * @return The x coordinate
     */
    public double getX() {
        return mX;
    }

    /**
     * Get the y coordinate
     * @return The y coordinate
     */

    public double getY() {
        return mY;
    }

    /**
     * Check if the CameraPoint2d is an angle
     * @return If the CameraPoint2d is an angle
     */

    public boolean isAngle() {
        return mIsAngle;
    }

    /**
     * Center the CameraPoint2d
     * 
     * @param resolution The resolution of the camera
     * @param xInverted If the x coordinate is inverted
     * @param yInverted If the y coordinate is inverted
     */
    public void center(Resolution resolution, boolean xInverted, boolean yInverted) {
        if(!mIsAngle) {
            mX = VisionUtil.centerPixels((int)mX, (double)resolution.getX(), xInverted);
            mY = VisionUtil.centerPixels((int)mY, (double)resolution.getY(), yInverted);
        } else {
            System.out.println("Pair is not in pixels");
        }
    }


    /**
     * Converts the CameraPoint2d to an angle
     * @param fov The field-of-view of the camera
     * @param resolution The resolution of the camera
     */
    public void toAngle(FOV fov, Resolution resolution) {
        if(!mIsAngle) {
            mX = VisionUtil.pixelsToAngles(mX, fov.getX(), resolution.getX());
            mY = VisionUtil.pixelsToAngles(mY, fov.getY(), resolution.getY());
        }
    }

    
    /**
     * Converts the CameraPoint2d to pixel coordinates
     * @param fov The field-of-view of the camera
     * @param resolution The resolution of the camera
     */
    public void toPixels(FOV fov, Resolution resolution) {
        if(mIsAngle) {
            mX = VisionUtil.anglesToPixels(mX, fov.getX(), resolution.getX());
            mY = VisionUtil.anglesToPixels(mY, fov.getY(), resolution.getY());
        }
    }


    /**
     * Stores camera parameters for point
     * @param fov
     * @param resolution
     */
    public void config(FOV fov, Resolution resolution) {
        mFov = fov;
        mRes = resolution;
    }

    
    /**
     * Converts the CameraPoint2d to an angle using the FOV and Resolution set globally with {@link CameraPoint2d#config(FOV, Resolution)}
     */
    public void toAngle() {
        if(mFov == null || mRes == null) {
            return;
        }
        if(!mIsAngle) {
            mX = VisionUtil.pixelsToAngles(mX, mFov.getX(), mRes.getX());
            mY = VisionUtil.pixelsToAngles(mY, mFov.getY(), mRes.getY());
        }
    }

    
    /**
     * Converts the CameraPoint2d to pixel coordinates using the FOV and Resolution set globally with {@link CameraPoint2d#config(FOV, Resolution)}
     */
    public void toPixels() {
        if(mFov == null || mRes == null) {
            return;
        }
        if(mIsAngle) {
            mX = VisionUtil.anglesToPixels(mX, mFov.getX(), mRes.getX());
            mY = VisionUtil.anglesToPixels(mY, mFov.getY(), mRes.getY());
        }
    }

}