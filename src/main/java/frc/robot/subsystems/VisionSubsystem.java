/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.vision.CameraPoint2d;
import frc.lib.vision.FOV;
import frc.lib.vision.Resolution;
import frc.lib.vision.VisionUtil;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    
    private final NetworkTable mLimelight;
    private final NetworkTableInstance mInstance;
    private final FOV mFov;
    private final Resolution mRes;
    private double mXOffset;
    private double mYOffset;

    // TODO: Turn LEDs on and off when vision is used

    /**
     * Creates a new VisionSubsystem
     */
    public VisionSubsystem(FOV fov, Resolution res) {
        mInstance = NetworkTableInstance.getDefault();
        
        mLimelight = mInstance.getTable("limelight");

        mFov = fov;
        mRes = res;

        setLED(false);
        setYOffset(Constants.Vision.kLimelightAngle);
    }

    @Override
    public void periodic() {
        
    }

    public void setYOffset(double offset) {
        mYOffset = offset;
    }

    public void setXOffset(double offset) {
        mXOffset = offset;
    }

    /**
     * 
     * @return If a target has been found
     */
    public boolean targetFound() {
        return mLimelight.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * 
     * @return The x location of the target in degrees
     */
    public double getX() {
        return mLimelight.getEntry("tx").getDouble(0.0) + mXOffset;
    }

    /**
     * 
     * @return The y location of the target in degrees
     */
    public double getY() {
        return mLimelight.getEntry("ty").getDouble(0.0) + mYOffset;
    }

    /**
     * 
     * @return The width of the target in pixels
     */
    public double getTargetWidth() {
        return mLimelight.getEntry("thor").getDouble(0.0);
    }

    /**
     * 
     * @return The height of the target in pixels
     */
    public double getTargetHeight() {
        return mLimelight.getEntry("tvert").getDouble(0.0);
    }

    public double getDistanceWidth(double targetWidth) {
      double x = getX();
      double width = getTargetWidth();
      double angleLeft = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, mFov.getX(), mRes.getX())-width/2.0, mFov.getX(), mRes.getX());
      double angleRight = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, mFov.getX(), mRes.getX())+width/2.0, mFov.getX(), mRes.getX());
      double widthAngle = angleRight-angleLeft;
      double distance = (targetWidth / 2.0) / (Math.tan(Math.toRadians((widthAngle / 2.0))));

      return distance;
    }

    public double getDistanceHeight() {
        return Constants.Vision.kVerticalOffset / Math.tan(Math.toRadians(getTargetCenter().getY()));
    }

    public double getTargetAngle() {
        double targetWidth = VisionUtil.pixelsToAngles(getTargetWidth(), Constants.Vision.kLimelightFOV.getX(), Constants.Vision.kLimelightResolution.getX());
        double maxTargetWidth = Math.toDegrees(Math.atan(Constants.Vision.kTargetWidth / (2.0 * getDistanceHeight())));
        return Math.acos(targetWidth/maxTargetWidth);
    }
    
    public void setPipeline(int pipeline) {
        mLimelight.getEntry("pipeline").setNumber(pipeline);
    }

    public void defaultLED() {
        mLimelight.getEntry("ledMode").setNumber(0);
    }

    public void blinkLED() {
        mLimelight.getEntry("ledMode").setNumber(2);
    }

    public void setLED(boolean on) {
        if (on) {
            mLimelight.getEntry("ledMode").setNumber(1);
        } else {
            mLimelight.getEntry("ledMode").setNumber(3);
        }
    }

    public void camMode(boolean driverCam) {
        mLimelight.getEntry("camMode").setNumber(driverCam ? 1 : 0);
    }

    public void setPIP(boolean driverCam) {
        mLimelight.getEntry("stream").setNumber(driverCam ? 2 : 1);
    }

    public CameraPoint2d[] getCorners() {
        double[] x = mLimelight.getEntry("tcornx").getDoubleArray(new double[0]);
        double[] y = mLimelight.getEntry("tcorny").getDoubleArray(new double[0]);
        CameraPoint2d[] corners = new CameraPoint2d[x.length];
        for (int i = 0; i < x.length; i++) {
            corners[i] = new CameraPoint2d(x[i], y[i], true);
        }
        return corners;
    }

    public CameraPoint2d[] getTopCorners() {
        CameraPoint2d[] corners = getCorners();
        if(corners.length < 2) {
            return new CameraPoint2d[0];
        }
        CameraPoint2d[] topCorners = new CameraPoint2d[2];
        for (int i = 0; i < corners.length; i++) {
            if(corners[i].getY() > topCorners[1].getY()) {
                topCorners[i] = corners[i];
            }
            if(topCorners[1].getY() > topCorners[0].getY()) {
                CameraPoint2d upperPoint = topCorners[1];
                topCorners[1] = topCorners[0];
                topCorners[0] = upperPoint;
            }
        }
        return topCorners;
    }

    public CameraPoint2d getTargetCenter() {
        return new CameraPoint2d((getTopCorners()[0].getX()+getTopCorners()[1].getX())/2.0, (getTopCorners()[0].getY()+getTopCorners()[1].getY())/2.0, true);
    }
}