/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.util.Units;
import frc.lib.config.FeedbackConstants;
import frc.lib.config.MotionMagicConfig;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.lib.vision.FOV;
import frc.lib.vision.Resolution;

public final class Constants {

    // Button Board Constants
    public static class ButtonBoard {
        public static final int kPort = 1;
        public static final int kSpinUp = 1;
        public static final int kAim = 2;
        public static final int kFeedShooter = 3;
        public static final int kShootSequence = 4;
        public static final int kFrontIntake = 5;
        public static final int kBackIntake = 6;
        public static final int kFrontOutput = 7;
        public static final int kBackOutput = 8;
        public static final int kSpinWOF = 9;
        public static final int kClimbExtend = 10;
        public static final int kClimbVent = 11;
        public static final int kClimbRetract = 12;
        public static final int kWOFLeft = 13;
        public static final int kWOFLeftSmall = 14;
        public static final int kWOFRight = 15;
        public static final int kWOFRightSmall = 16;
    }

    public static class Drive {

        // Drive Motor Ports
        public static final int kLeftDriveMotorPort = 0;
        public static final int kLeftDriveMotorFollowerPort = 1;
        public static final int kRightDriveMotorPort = 2;
        public static final int kRightDriveMotorFollowerPort = 3;

        // Driving Constants
        public static final double kDeadbandThreshold = 0.02;
        public static final int kJoystickPort = 0;
        public static final int kThrottleAxis = 1;
        public static final int kTurnAxis = 2;
        public static final int kTrimAxis = 0;
        public static final double kMaxPower = 1.0;
        public static final double kTurnRatio = 0.25;
        public static final double kTrimRatio = 0.5;
        public static final boolean kThrottleInverted = true;
        public static final boolean kTurnInverted = false;
        public static final boolean kTrimInverted = false;
        public static final double kCreepSpeed = 0.15;

        // Drive System Constants
        public static final boolean kRightDriveInverted = false;
        public static final boolean kDriveInverted = false;
        public static final boolean kRightInverted = false;
        public static final boolean kLeftInverted = true;
        public static final int kDriveSlot = 0;
        public static final FeedbackConstants kDriveFeedbackConstants = new FeedbackConstants(0.02, 0, 0.02, 0.05, 200);
        public static final MotorConfig kDriveConfig = new MotorConfig(40, 0, 0, FeedbackDevice.IntegratedSensor, 2048, 12.75*(48.0/42.0), 0.001);  // TODO: Tune PID

        // Trajectory Following Constants
        public static final SimpleMotorFeedforward kDriveFeedforward = new SimpleMotorFeedforward(0.289, 2.42, 0.361);
        public static final PIDController kPathPID = new PIDController(0.0287, 0,  0);
        public static final double kTrackWidth = 0.511;
        public static final double kB = 2.0;
        public static final double kZeta = 0.7;
        public static final double kDriveWheelDiameter = Units.inchesToMeters(8);
        public static final double kVelSampleTime = 0.1;

        // Trajectory Generation Constants
        public static final double kMaxVelocity = 0.75;  // TODO: Determine correct value
        public static final double kMaxAcceleration = 0.75;  // TODO: Determine correct value

        // Aiming Constants
        public static final double kMaxAimPower = 0.1;

    }

    public static class Robot {

        public static final double kMinBatteryVoltage = 10;  // TODO: Determine correct value

    }
    

    public static class Config {

        // Motion Magic Constants
        public static final int kMotionMagicPeriod = 10;

    }

    public static class Gyro {

        // Gyro Constants
        public static final boolean kGyroReversed = false;  // TODO: Determine correct value

    }

    public static class Dashboard {

        // Dashboard Constants
        public static final double kUpdateRate = 0.5;  // TODO: Determine correct value

    }

    
    public static class Vision {

        // Vision Constants
        public static final FOV kLimelightFOV = new FOV(59.6, 45.7);
        public static final Resolution kLimelightResolution = new Resolution(960, 720);
        public static final double kLimelightAngle = 16.5;  //TODO: Determine correct value
        public static final double kLimelightHeight = .794;  //TODO: Determine final height
        public static final double kTargetHeight = 2.5;
        public static final double kVerticalOffset = kTargetHeight - kLimelightHeight;
        public static final double kTargetWidth = 1.0;
        public static final double kTapeWidth = (4.0 / Math.sqrt(3)) * 0.0254;
        public static final double kPAim = 0.02;
        public static final double kIAim = 0.0;
       // public static final double kIAim = 0.0025;
        public static final double kDAim = 0.0065;
        public static final double kDampening = 0.0;
        public static final double kAimOffset = 0.0;
    }

    public static class Camera {
         
        // Camera Constants
        public static final Resolution kCameraResolution = new Resolution(160, 120);
        public static final int kCameraFPS = 14;
        public static final int kCameraCompression = 75;
    }   

    public static class Shooter {
        
        // Shooter Constants
        public static final int kLeftShooterMotor = 11;
        public static final int kRightShooterMotor = 10;
        public static final FeedbackConstants kShooterFeedbackConstants = new FeedbackConstants(0.1, 0.001, 0.0, 0.0463, 10000);
        public static final MotorConfig kShooterConfig = new MotorConfig(40, 0, 0, FeedbackDevice.IntegratedSensor, 2048, 1.0, 0.001); // TODO: Redo current on shooter
        public static final double kStableRPMThreshold = 100;
        public static final double kShooterVelocitySampleTime = 0.1;
        public static final double kShooterStableCurrentLimit = 10;
        public static final int kMaxShooterSlewRate = 4000;  // TODO: Determine correct value
        public static final int kShooterSlot = 0;
        public static final boolean kShooterLeftInveted = true;
        public static final boolean kShooterRightInverted = false;
    
    }

    public static class WOF {

        // Color Wheel Constants
        public static final double kWOFDiameter = 32;
        public static final double kWOFWheelDiameter = 7.5;
        public static final FeedbackConstants kWOFFeedbackConstants = new FeedbackConstants(0.01, 0, 0.01, 0.008, 0);
        public static final MotorConfig kWOFConfig = new MotorConfig(10, 20, 500, FeedbackDevice.QuadEncoder, 4096, 20.0, 0.001);  // TODO: Tune PID
        public static final MotionMagicConfig kWOFMMConfig = new MotionMagicConfig((int)EncoderUtil.fromRPM(300, kWOFConfig.getEpr(), kWOFConfig.getGearRatio(), Constants.Drive.kVelSampleTime), (int)EncoderUtil.fromRPM(600, kWOFConfig.getEpr(), kWOFConfig.getGearRatio(), Constants.Drive.kVelSampleTime), 0);  // TODO: Determine correct values
        public static final double kRamp = 0.1;
        public static final int kWOFSlot = 0;
        public static final int kWOFMotor = 5;
        public static final double kWOFThreshold = 0.02;
        public static final double kWOFRotations = 3.5;

    }

    public static class Transport {

        // Transport Constants
        public static final int kTransportTalon = 6;
        public static final double kPulleyDiameter = 2.75;  //TODO: Get correct diameter
        public static final double kVelocitySampleTime = 0.1;
        public static final FeedbackConstants kTransportFeedbackConstants = new FeedbackConstants(0.15, 0, 0.05, 0, 0);
        public static final FeedbackConstants kTransportVelocityFeedbackConstants = new FeedbackConstants(0.01, 0, 1.0, 0.030536, 0);
        public static final MotorConfig kTransportConfig = new MotorConfig(15, 25, 500, FeedbackDevice.QuadEncoder, 4096, 20.0, 0.001);
        public static final MotionMagicConfig kTransportMMConfig = new MotionMagicConfig((int)EncoderUtil.fromVelocity(40.0, kTransportConfig.getEpr(), kTransportConfig.getGearRatio(), kPulleyDiameter, kVelocitySampleTime),
            (int)EncoderUtil.fromVelocity(160.0, kTransportConfig.getEpr(), kTransportConfig.getGearRatio(), kPulleyDiameter, kVelocitySampleTime),
            0);
        public static final int kTransportSlot = 0;
        public static final int kTransportVelocitySlot = 1;
        public static final double kTransportVelocity = 25;
        public static final double kTransportThreshold = 0.125;
        public static final int kTransportDebounceSize = 5;
        public static final double kTransportMargin = 5.0;

        // TOF Sensors
        public static final int kIntakeTOF = 1;  // TODO: Determine correct value
        public static final int kShotoerTOF = 0;  // TODO: Determine correct value
        public static final double kMaxDistance = 130.0;  // TODO: Determine correct value
        public static final double kMaxDeviation = 5.0;
        public static final RangingMode kRangingMode = RangingMode.Short;
        public static final double kTOFSampleTime = 20;

    }


    public static class Intake {

        // Intake Constants
        public static final int kIntakeTalon = 4;
        public static final double kIntakePower = 0.75;
        public static final boolean kIntakeInverted = false;

    }

    public static class Climb {

        // Climb Constants
        public static final int kLeftClimbTop = 3;
        public static final int kLeftClimbBottom = 2;
        public static final int kRightClimbTop = 1;
        public static final int kRightClimbBottom = 0;
    
    }

}
