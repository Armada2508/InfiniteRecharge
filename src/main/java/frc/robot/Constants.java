/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.lib.config.MotionMagicConfig;
import frc.lib.config.MotorConfig;
import frc.lib.vision.FOV;
import frc.lib.vision.Resolution;

public final class Constants {

    // Drive Motor Ports
    public static final int kRightMotorPort = 3;
    public static final int kRightMotorFollowerPort = 4;
    public static final int kLeftMotorPort = 7;
    public static final int kLeftMotorFollowerPort = 8;

    // Driving Constants
    public static final double kDeadbandThreshold = 0.06;
    public static final int kJoystickPort = 0;
    public static final int kThrottleAxis = 1;
    public static final int kTurnAxis = 2;
    public static final int kTrimAxis = 0;
    public static final double kMaxPower = 1.0;
    public static final double kTurnRatio = 0.5;
    public static final double kTrimRatio = 0.5;
    public static final boolean kThrottleInverted = true;
    public static final boolean kTurnInverted = false;
    public static final boolean kTrimInverted = false;

    // Drive System Constants
    public static final boolean kRightInverted = false;
    public static final boolean kDriveInverted = false;
    public static final boolean kRightSensorInverted = false;
    public static final boolean kLeftSensorInverted = true;
    public static final int kDriveSlot = 0;
    public static final MotorConfig kDriveConfig = new MotorConfig(0, 0, 0, 0, 200, 40, FeedbackDevice.IntegratedSensor, 0.001);

    // Trajectory Following Constants
    public static final SimpleMotorFeedforward kDriveFeedforward = new SimpleMotorFeedforward(0.129, 1.35, 3.52);
    public static final double kTrackWidth = 0.5612243747769792;
    public static final double kB = 2.0;
    public static final double kZeta = 0.7;
    public static final int kTicksPerRev = 4096;
    public static final double kWheelDiameter = 0.1524;
    public static final double kGearRatio = 42.0/38.0;
    public static final double kVelSampleTime = 0.1;
    public static final double kMaxMotorVoltage = 0.0;

    // Trajectory Generation Constants
    public static final double kMaxVelocity = 0.75;
    public static final double kMaxAcceleration = 0.75;

    // Motion Magic Constants
    public static final int kMotionMagicPeriod = 10;

    // Gyro Constants
    public static final boolean kGyroReversed = false;

    // Dashboard Constants
    public static final double kUpdateRate = 0.5;

    // Vision Constants
    public static final FOV kLimelightFOV = new FOV(59.6, 45.7);
    public static final Resolution kLimelighResolution = new Resolution(320, 240);

    // Shooter Constants
    public static final int kShooterEncoderUnitsPerRev = 2048;
    public static final double kShooterGearRatio = 1.0;
    public static final double kShooterVelocitySampleTime = 0.1;
    public static final MotorConfig kShooterConfig = new MotorConfig(0, 0, 0, 0, 0, 40, FeedbackDevice.QuadEncoder, 0.001);
    public static final int kMaxSlewRate = 5000;
    public static final int kShooterSlot = 0;

    // Color Wheel Constants
    public static final double kWOFDiameter = 13;
    public static final int kWOFEncoderUnitsPerRev = 1024;
    public static final double kWOFGearRatio = 20.0;
    public static final double kWOFWheelDiameter = 4.0;
    public static final MotorConfig kWOFConfig = new MotorConfig(0, 0, 0, 0, 0, 30, FeedbackDevice.QuadEncoder, 0.001);
    public static final MotionMagicConfig kWOFMMConfig = new MotionMagicConfig(0, 0, 0);
    public static final int kWOFSlot = 0;

    // Transport Constants
    public static final double kPulleyDiameter = 2.75;
    public static final int kTransportEncoderUnitsPerRev = 1024;
    public static final double kTransportGearRatio = 20.0;
    public static final MotorConfig kTransportConfig = new MotorConfig(0, 0, 0, 0, 0, 30, FeedbackDevice.QuadEncoder, 0.001);
    public static final MotionMagicConfig kTransportMMConfig = new MotionMagicConfig(0, 0, 0);
    public static final int kTransportSlot = 0;
    

}
