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
import frc.lib.util.Util;
import frc.lib.vision.CameraPoint2d;
import frc.lib.vision.FOV;
import frc.lib.vision.Resolution;
import frc.lib.vision.VisionUtil;
import frc.robot.Constants;
import frc.robot.enums.CamMode;
import frc.robot.enums.StreamingMode;

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
        setCamMode(CamMode.DRIVER);
        setStreamingMode(StreamingMode.PIPPRIM);
    }

    /**
     * Reset all parameters(LEDs, Pipeline, Driver Cam, PIP)
     */
    public void reset() {
        setLED(false);
        setPipeline(0);
        setCamMode(CamMode.DRIVER);
        setStreamingMode(StreamingMode.PIPPRIM);
    }

    /**
     * 
     * @return If a target has been found
     */
    public boolean targetFound() {
        return Util.epsilonEquals(mLimelight.getEntry("tv").getDouble(0.0), 1.0);
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
        return mLimelight.getEntry("ty").getDouble(0.0) + mYOffset*Math.cos(Math.toRadians(getX()));
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
     * Get the distance to the target based on its width
     * @param targetWidth The physical width of the target
     * @return The distance to the target in the same units as {@code targetWidth}
     */
    public double getDistanceWidth(double targetWidth) {
      if(!targetFound()) return 0.0;
      return VisionUtil.getDistanceWidth(targetWidth, mRes, mFov, getTargetWidth(), getX());
    }

    /**
     * Get the distance to the target based on its height
     * @param delta The difference between the height of the camera and the height of the target
     * @return The distance to the target in the same units as {@code delta}
     */
    public double getDistanceHeight(double delta) {
        if(!targetFound()) return 0.0;
        return VisionUtil.getDistanceHeight(delta, getTargetCenter().getY());
    }

    /**
     * Get the skewed angle of the target
     * @return The angle of the target in degrees
     */
    public double getTargetAngle() {
        CameraPoint2d[] topPoints = getTopCorners();
        if(!targetFound() || topPoints.length > 1) {
            return 0;
        }
        return VisionUtil.getSkewAngle(topPoints[0], topPoints[1], Constants.Vision.kTargetWidth - Constants.Vision.kTapeWidth, getDistanceHeight(Constants.Vision.kVerticalOffset));
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
     * Set the camera up for driver or computer vision use
     * @param camMode The camera mode
     */
    public void setCamMode(CamMode camMode) {
        mLimelight.getEntry("camMode").setNumber(camMode ==  CamMode.DRIVER ? 1 : 0);
    }

    /**
     * Sets the streaming mode
     * @param streamingMode The streaming mode
     */
    public void setStreamingMode(StreamingMode streamingMode) {
        double mode = 0;
        switch (streamingMode) {
            case SBS:
                mode = 0;
                break;
            case PIPPRIM:
                mode = 1;
                break;
            case PIPAUX:
                mode = 2;
                break;
        }
        mLimelight.getEntry("stream").setNumber(mode);
    }

    /**
     * @return The camera mode
     */
    public CamMode getCamMode() {
        switch (mLimelight.getEntry("CamMode").getNumber(1).intValue()) {
            case 1:
                return CamMode.DRIVER;
            case 0:
                return CamMode.CV;
            default:
                return CamMode.DRIVER;
        }
    }

    /**
     * @return The streaming mode
     */
    public StreamingMode getStreamingMode() {
        switch (mLimelight.getEntry("CamMode").getNumber(1).intValue()) {
            case 2:
                return StreamingMode.PIPAUX;
            case 1:
                return StreamingMode.PIPPRIM;
            case 0:
                return StreamingMode.SBS;
            default:
                return StreamingMode.SBS;
        }
    }

    /**
     * @return All detected corners
     */
    public CameraPoint2d[] getCorners() {
        double[] xy = mLimelight.getEntry("tcornxy").getDoubleArray(new double[0]);
        return VisionUtil.parseCorners(xy, Constants.Vision.kLimelightResolution, Constants.Vision.kLimelightFOV, mXOffset, mYOffset);
    }

    /**
     * @return The top two corners, the left point having index 0 and the right having index 1
     */
    public CameraPoint2d[] getTopCorners() {
        CameraPoint2d[] corners = getCorners();
        return VisionUtil.getTopCorners(corners);
    }

    /**
     * @return The center of the target
     */
    public CameraPoint2d getTargetCenter() {
        CameraPoint2d[] corners = getTopCorners();
        if(corners.length < 2) {
            return new CameraPoint2d(0, 0);
        }
        return CameraPoint2d.midpoint(corners[0], corners[1]);
    }
}