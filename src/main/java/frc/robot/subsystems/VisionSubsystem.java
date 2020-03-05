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
    public VisionSubsystem() {
        mInstance = NetworkTableInstance.getDefault();
        
        mLimelight = mInstance.getTable("limelight");

        mFov = Constants.Vision.kLimelightFOV;
        mRes = Constants.Vision.kLimelightResolution;
        
        setup();
    }

    @Override
    public void periodic() {
        System.out.println(getDistanceHeight() + "; " + getTargetCenter().getY());
    }

    public void setYOffset(double offset) {
        mYOffset = offset;
    }

    public void setXOffset(double offset) {
        mXOffset = offset;
    }

    public void setup() {
        setLED(true);
        setYOffset(Constants.Vision.kLimelightAngle);
        setPipeline(0);
        camMode(false);
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

    public double getDistanceHeight() {
        if(!targetFound()) return 0.0;
        return Constants.Vision.kVerticalOffset / Math.tan(Math.toRadians(getTargetCenter().getY()));
    }

    public double getTargetAngle() {
        CameraPoint2d[] topPoints = getTopCorners();
        double targetWidth = topPoints[1].getX() - topPoints[0].getX();
        double maxTargetWidth = Math.toDegrees(Math.atan((Constants.Vision.kTargetWidth - Constants.Vision.kTapeWidth ) / (2.0 * getDistanceHeight())));
        int directionMultiplier = (topPoints[0].getY() > topPoints[1].getY()) ? -1 : 1;
        return Math.acos(targetWidth/maxTargetWidth) * directionMultiplier;
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
        if (!on) {
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

    public CameraPoint2d[] getTopCorners() {
        CameraPoint2d[] corners = getCorners();
        if(corners.length < 2) {
            return new CameraPoint2d[0];
        }
        CameraPoint2d[] topCorners = new CameraPoint2d[2];
        for (int i = 0; i < 2; i++) {
            topCorners[i] = corners[i];
        }
        for (int i = 0; i < corners.length; i++) {
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

    public CameraPoint2d getTargetCenter() {
        if(getTopCorners().length < 2) {
            return new CameraPoint2d(0, 0);
        }
        return new CameraPoint2d((getTopCorners()[0].getX()+getTopCorners()[1].getX())/2.0, (getTopCorners()[0].getY()+getTopCorners()[1].getY())/2.0, true);
    }
}