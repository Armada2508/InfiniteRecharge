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
import frc.lib.vision.FOV;
import frc.lib.vision.Resolution;
import frc.lib.vision.VisionUtil;

public class VisionSubsystem extends SubsystemBase {
    
    private final NetworkTable mLimelight;
    private final NetworkTableInstance mInstance;
    private final FOV mFov;
    private final Resolution mRes;

    // TODO: Turn LEDs on and off when vision is used

    /**
     * Creates a new VisionSubsystem
     */
    public VisionSubsystem(FOV fov, Resolution res) {
        mInstance = NetworkTableInstance.getDefault();
        
        mLimelight = mInstance.getTable("limelight");

        mFov = fov;
        mRes = res;
    }

    @Override
    public void periodic() {
        
    }

    public void setYOffset(double offset) {

    }

    public void setXOffset(double offset) {
        
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
        return mLimelight.getEntry("tx").getDouble(0.0);
    }

    /**
     * 
     * @return The y location of the target in degrees
     */
    public double getY() {
        return mLimelight.getEntry("ty").getDouble(0.0);
    }

    /**
     * 
     * @return The width of the target in pixels
     */
    public double getWidth() {
        return mLimelight.getEntry("thor").getDouble(0.0);
    }

    /**
     * 
     * @return The height of the target in pixels
     */
    public double getHeight() {
        return mLimelight.getEntry("tvert").getDouble(0.0);
    }

    public double getDistanceWidth(double targetWidth) {
      double x = getX();
      double width = getWidth();
      double angleLeft = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, mFov.getX(), mRes.getX())-width/2.0, mFov.getX(), mRes.getX());
      double angleRight = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, mFov.getX(), mRes.getX())+width/2.0, mFov.getX(), mRes.getX());
      double widthAngle = angleRight-angleLeft;
      double distance = (targetWidth / 2.0) / (Math.tan(Math.toRadians((widthAngle / 2.0))));

      return distance;
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
}