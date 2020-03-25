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

    /**
     * Creates a new VisionSubsystem
     */
    public VisionSubsystem() {
        mInstance = NetworkTableInstance.getDefault();
        
        mLimelight = mInstance.getTable("limelight");

        mFov = Constants.Vision.kLimelightFOV;
        mRes = Constants.Vision.kLimelightResolution;
        
        setup();
    }

    @Override
    public void periodic() {
    }

    /**
     * Set the vertical offset of the limelight
     * @param offset The offset in degrees
     */
    public void setYOffset(double offset) {
        mYOffset = offset;
    }


    /**
     * Set the horizontal offset of the limelight
     * @param offset The offset in degrees
     */
    public void setXOffset(double offset) {
        mXOffset = offset;
    }

    /**
     * Setup the subsystem
     */
    public void setup() {
        setLED(false);
        setYOffset(Constants.Vision.kLimelightAngle);
        setPipeline(0);
        camMode(true);
        setPIP(true);
    }

    /**
     * Reset all parameters(LEDs, Pipeline, Driver Cam, PIP)
     */
    public void reset() {
        setLED(false);
        setPipeline(0);
        camMode(true);
        setPIP(true);
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

    /**
     * Get the distance to the target based on it's width
     * @param targetWidth The width of the target
     * @return The distance to the target in the same units as {@code targetWidth}
     */
    public double getDistanceWidth(double targetWidth) {
      if(!targetFound()) return 0.0;

      double x = getX();
      double width = getTargetWidth();
      double angleLeft = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, mFov.getX(), mRes.getX())-width/2.0, mFov.getX(), mRes.getX());
      double angleRight = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, mFov.getX(), mRes.getX())+width/2.0, mFov.getX(), mRes.getX());
      double widthAngle = angleRight-angleLeft; 
      double distance = (targetWidth / 2.0) / (Math.tan(Math.toRadians((widthAngle / 2.0))));

      return distance;
    }

    /**
     * Get the distance to the target based on it's height
     * @param height The height of the target
     * @return The distance to the target in the same units as {@code height}
     */
    public double getDistanceHeight(double height) {
        if(!targetFound()) return 0.0;
        return height / Math.tan(Math.toRadians(getTargetCenter().getY()));
    }

    /**
     * Get the skewed angle of the target
     * @return The angle of the target in degrees
     */
    public double getTargetAngle() {
        CameraPoint2d[] topPoints = getTopCorners();
        double targetWidth = topPoints[1].getX() - topPoints[0].getX();
        double maxTargetWidth = Math.toDegrees(Math.atan((Constants.Vision.kTargetWidth - Constants.Vision.kTapeWidth ) / (2.0 * getDistanceHeight(Constants.Vision.kVerticalOffset))));
        int directionMultiplier = (topPoints[0].getY() > topPoints[1].getY()) ? -1 : 1;
        return Math.toDegrees(Math.acos(targetWidth/maxTargetWidth)) * directionMultiplier;
    }
    
    /**
     * Set the current pipeline
     * @param pipeline The pipeline to use(0-9)
     */
    public void setPipeline(int pipeline) {
        mLimelight.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Set the LED mode to the default mode specified in the Web UI
     */
    public void defaultLED() {
        mLimelight.getEntry("ledMode").setNumber(0);
    }

    /**
     * Blink the LEDs
     */
    public void blinkLED() {
        mLimelight.getEntry("ledMode").setNumber(2);
    }

    /**
     * Set the LEDs to on or off
     * @param on If the LEDs are on
     */
    public void setLED(boolean on) {
        if (!on) {
            mLimelight.getEntry("ledMode").setNumber(1);
        } else {
            mLimelight.getEntry("ledMode").setNumber(3);
        }
    }

    /**
     * Set the camera to 
     * @param driverCam If the limelight is in driving mode
     */
    public void camMode(boolean driverCam) {
        mLimelight.getEntry("camMode").setNumber(driverCam ? 1 : 0);
    }

    /**
     * Set the streaming mode to Side-by-Side
     */
    public void setSBS() {
        mLimelight.getEntry("stream").setNumber(0);
    }

    /**
     * Sets the streaming mode to Picture-in-Picture
     * @param secondaryCam If the secondary camera is bigger than the limelight camera
     */
    public void setPIP(boolean secondaryCam) {
        mLimelight.getEntry("stream").setNumber(secondaryCam ? 2 : 1);
    }

    /**
     * @return All detected corners
     */
    public CameraPoint2d[] getCorners() {
        double[] xy = mLimelight.getEntry("tcornxy").getDoubleArray(new double[0]);
        CameraPoint2d[] corners = new CameraPoint2d[xy.length/2];
        for (int i = 0; i < xy.length; i+=2) {
            corners[i/2] = new CameraPoint2d(xy[i], xy[i+1], false);
            corners[i/2].center(Constants.Vision.kLimelightResolution, false, true);
            corners[i/2].config(Constants.Vision.kLimelightFOV, Constants.Vision.kLimelightResolution);
            corners[i/2].toAngle();
            corners[i/2].setY(corners[i/2].getY() + mYOffset*Math.cos(Math.toRadians(corners[i/2].getX())));
            corners[i/2].setX(corners[i/2].getX() + mXOffset);
        }
        return corners;
    }

    /**
     * @return The top two corners, the left point having index 0 and the right having index 1
     */
    public CameraPoint2d[] getTopCorners() {
        CameraPoint2d[] corners = getCorners();
        if(corners.length < 2) {
            return new CameraPoint2d[0];
        }
        CameraPoint2d[] topCorners = new CameraPoint2d[2];
        for (int i = 0; i < 2; i++) {
            topCorners[i] = corners[i];
        }
        for (int i = topCorners.length; i < corners.length; i++) {
            if(corners[i].getY() > topCorners[1].getY()) {
                topCorners[1] = corners[i];
            }
            if(topCorners[1].getY() > topCorners[0].getY()) {
                CameraPoint2d upperPoint = topCorners[1];
                topCorners[1] = topCorners[0];
                topCorners[0] = upperPoint;
            }
        }
        if(topCorners[0].getX() > topCorners[1].getX()) {
            CameraPoint2d leftPoint = topCorners[1];
            topCorners[1] = topCorners[0];
            topCorners[0] = leftPoint;
        }
        return topCorners;
    }

    /**
     * @return The center of the target
     */
    public CameraPoint2d getTargetCenter() {
        if(getTopCorners().length < 2) {
            return new CameraPoint2d(0, 0);
        }
        return new CameraPoint2d((getTopCorners()[0].getX()+getTopCorners()[1].getX())/2.0, (getTopCorners()[0].getY()+getTopCorners()[1].getY())/2.0, true);
    }
}