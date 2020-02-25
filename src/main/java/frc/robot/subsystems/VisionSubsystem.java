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
    
    private final NetworkTable m_limelight;
    private final NetworkTableInstance m_instance;
    private final FOV m_fov;
    private final Resolution m_res;
    
    /**
     * Creates a new VisionSubsystem
     */
    public VisionSubsystem(FOV fov, Resolution res) {
        m_instance = NetworkTableInstance.getDefault();
        
        m_limelight = m_instance.getTable("limelight");

        m_fov = fov;
        m_res = res;
    }

    @Override
    public void periodic() {
        
    }

    /**
     * 
     * @return If a target has been found
     */
    public boolean targetFound() {
        return m_limelight.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * 
     * @return The x location of the target in degrees
     */
    public double getX() {
        return m_limelight.getEntry("tx").getDouble(0.0);
    }

    /**
     * 
     * @return The y location of the target in degrees
     */
    public double getY() {
        return m_limelight.getEntry("ty").getDouble(0.0);
    }

    /**
     * 
     * @return The width of the target in pixels
     */
    public double getWidth() {
        return m_limelight.getEntry("thor").getDouble(0.0);
    }

    /**
     * 
     * @return The height of the target in pixels
     */
    public double getHeight() {
        return m_limelight.getEntry("tvert").getDouble(0.0);
    }

    public double getDistanceWidth(double targetWidth) {
      double x = getX();
      double width = getWidth();
      double angleLeft = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, m_fov.getX(), m_res.getX())-width/2.0, m_fov.getX(), m_res.getX());
      double angleRight = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, m_fov.getX(), m_res.getX())+width/2.0, m_fov.getX(), m_res.getX());
      double widthAngle = angleRight-angleLeft;
      double distance = (targetWidth / 2.0) / (Math.tan(Math.toRadians((widthAngle / 2.0))));

      return distance;
    }

    

    public void defaultLED() {
        m_limelight.getEntry("ledMode").setNumber(0);
    }

    public void blinkLED() {
        m_limelight.getEntry("ledMode").setNumber(2);
    }

    public void setLED(boolean on) {
        if (on) {
            m_limelight.getEntry("ledMode").setNumber(1);
        } else {
            m_limelight.getEntry("ledMode").setNumber(3);
        }
    }
}