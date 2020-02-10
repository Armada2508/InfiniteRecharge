/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.vision.FOV;
import frc.lib.vision.Resolution;

public final class Constants {

    // Motor Ports
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

    // Trajectory Following Constants
    public static final double kS = 1.35;
    public static final double kV = 3.52;
    public static final double kA = 0.129;
    public static final double kTrackWidth = 0.5612243747769792;
    public static final double kB = 2.0;
    public static final double kZeta = 0.7;
    public static final int kTicksPerRev = 4096;
    public static final double kWheelDiameter = 0.1524;
    public static final double kGearRatio = 42.0/38.0;
    public static final double kVelSampleTime = 0.1;
    public static final double kMaxMotorVoltage = 0.0;

    // Trajectory Generation constants
    public static final double kMaxVelocity = 0.75;
    public static final double kMaxAcceleration = 0.75;

    // PID
    public static final int kSlot = 0;
    public static final double kF = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final int kMIA = 200;

    // Gyro Settings
    public static final boolean kGyroReversed = false;

    // Dashboard Settings
    public static final double kUpdateRate = 0.5;

    // Vision Test Settings
    public static final double kTurn = 0.02;
    public static final double kThrottle = 0.03;
    public static final double kMaxFollowOutput = 0.6;
    public static final double kTargetWidth = 4.0;
    public static final double kTargetDistance = 60;

    // Vision Settings
    public static final FOV kLimelightFOV = new FOV(59.6, 45.7);
    public static final Resolution kLimelighResolution = new Resolution(320, 240);

    // Pneumatic Settings
    public static final int kLShift = 0;
    public static final int kRShift = 1;

    // Shooter Settings
    public static final int kShooterEncoderUnitsPerRev = 2048;
    public static final double kShooterGearRatio = 1.0;
    public static final double kShooterVelocitySampleTime = 0.1;
    

}
