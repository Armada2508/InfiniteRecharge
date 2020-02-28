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
import frc.lib.config.MotionMagicConfig;
import frc.lib.config.MotorConfig;
import frc.lib.vision.FOV;
import frc.lib.vision.Resolution;

//TODO: Put constants in sub-classes

public final class Constants {

    // Button Board Constants
    public static class ButtonBoard {
        public static final int port = 1;
        public static final int kSpinUp = 0;
        public static final int kAim = 1;
        public static final int kFeedShooter = 2;
        public static final int kShootSequence = 3;
        public static final int kFrontIntake = 4;
        public static final int kBackIntake = 5;
        public static final int kBothIntake = 6;
        public static final int kFrontOutput = 7;
        public static final int kBackOutput = 8;
        public static final int kSpinWOF = 9;
        public static final int kIncrementWOF = 10;
        public static final int kClimbExtend = 11;
        public static final int kClimbRetract = 12;
        public static final int kStop = 13;
        
    }

    public static class Drive {

        // Drive Motor Ports
        public static final int kLeftDriveMotorPort = 0;
        public static final int kLeftDriveMotorFollowerPort = 1;
        public static final int kRightDriveMotorPort = 2;
        public static final int kRightDriveMotorFollowerPort = 3;

        // Driving Constants
        public static final double kDeadbandThreshold = 0.06;  // TODO: Determine correct value
        public static final int kJoystickPort = 0;
        public static final int kThrottleAxis = 1;
        public static final int kTurnAxis = 2;
        public static final int kTrimAxis = 0;
        public static final double kMaxPower = 0.2;
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
        public static final MotorConfig kDriveConfig = new MotorConfig(0, 0, 0, 0, 200, 40, 0, 0, FeedbackDevice.IntegratedSensor, 0.001);  // TODO: Tune PID
        public static final PIDController kDrivePositionPID = new PIDController(0, 0, 0);  // TODO: Tune PID
        public static final double kCoolingTemp = 60.0;
        public static final double kCoolingDelay = 5.0;
        public static final int kCoolingSolenoid = 4;

        // Trajectory Following Constants
        public static final SimpleMotorFeedforward kDriveFeedforward = new SimpleMotorFeedforward(0.129, 1.35, 3.52);  // TODO: Redo drive characterization
        public static final double kTrackWidth = 0.5612243747769792;  // TODO: Find track width
        public static final double kB = 2.0;
        public static final double kZeta = 0.7;
        public static final int kDriveEncoderUnitsPerRev = 2048;
        public static final double kDriveWheelDiameter = 0.2032;
        public static final double kDriveGearRatio = 42.0/38.0;  // TODO: Determine correct value
        public static final double kVelSampleTime = 0.1;

        // Trajectory Generation Constants
        public static final double kMaxVelocity = 0.75;  // TODO: Determine correct value
        public static final double kMaxAcceleration = 0.75;  // TODO: Determine correct value

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
        public static final Resolution kLimelightResolution = new Resolution(320, 240);
        public static final double kLimelightAngle = 15.0;  //TODO: Determine correct value
        public static final double kLimelightHeight = .794;  //TODO: Determine final height
        public static final double kTargetHeight = 2.5;
        public static final double kVerticalOffset = kTargetHeight - kLimelightHeight;
        public static final double kTargetWidth = .997;
        public static final double kTapeWidth = 4.0/Math.sqrt(3);

        public static final double kPAim = 0.025;
        public static final double kIAim = 0.0;
        public static final double kDAim = 0.0;
    }

    public static class Shooter {
        
        // Shooter Constants
        public static final int kLeftShooterMotor = 11;
        public static final int kRightShooterMotor = 10;
        public static final int kShooterEncoderUnitsPerRev = 2048;
        public static final double kShooterGearRatio = 1.0;
        public static final double kShooterVelocitySampleTime = 0.1;
        public static final MotorConfig kShooterConfig = new MotorConfig(0.25, 0.001, 0.0, 0.05, 10000.0, 40, 0, 0, FeedbackDevice.IntegratedSensor, 0.001); // TODO: Redo PID on shooter
        public static final int kMaxShooterSlewRate = 2000;  // TODO: Determine correct value
        public static final int kShooterSlot = 0;
        public static final boolean kShooterLeftInveted = true;
        public static final boolean kShooterRightInverted = false;
    
    }

    public static class WOF {

        // Color Wheel Constants
        public static final double kWOFDiameter = 13;
        public static final int kWOFEncoderUnitsPerRev = 1024;
        public static final double kWOFGearRatio = 20.0;
        public static final double kWOFWheelDiameter = 4.0;
        public static final MotorConfig kWOFConfig = new MotorConfig(0, 0, 0, 0, 0, 10, 20, 500, FeedbackDevice.QuadEncoder, 0.001);  // TODO: Tune PID
        public static final MotionMagicConfig kWOFMMConfig = new MotionMagicConfig(0, 0, 0);  // TODO: Determine correct values
        public static final int kWOFSlot = 0;
        public static final int kWOFMotor = 5;
        public static final double kWOFThreshold = 0.02;
        public static final double kWOFRotations = 3.5;

    }

    public static class Transport {

        // Transport Constants
        public static final int kElevatorTalon = 9;
        public static final int kDiagonalTalon = 6;
        public static final double kPulleyDiameter = 2.75;
        public static final int kTransportEncoderUnitsPerRev = 1024;
        public static final double kTransportGearRatio = 20.0;
        public static final MotorConfig kTransportConfig = new MotorConfig(0, 0, 0, 0, 0, 15, 25, 500, FeedbackDevice.QuadEncoder, 0.001);
        public static final MotionMagicConfig kTransportMMConfig = new MotionMagicConfig(0, 0, 0);  // TODO: Determine correct values
        public static final int kTransportSlot = 0;

        // TOF Sensors
        public static final int kFIntakeTofID = 0;  // TODO: Determine correct value
        public static final int kBIntakeTofID = 1;  // TODO: Determine correct value
        public static final int kInterfaceTofID = 2;  // TODO: Determine correct value
        public static final int kShooterTofID = 3;  // TODO: Determine correct value
        public static final double kMaxDistance = 2.0;  // TODO: Determine correct value
        public static final double kMaxDeviation = 1000000.0;  // TODO: Determine correct value
        public static final RangingMode kRangingMode = RangingMode.Short;  // TODO: Determine best value
        public static final double kTOFSampleTime = 50;  // TODO: Determine best value

    }


    public static class Intake {

        // Intake Constants
        public static final int kFrontIntakeTalon = 8;
        public static final int kBackIntakeTalon = 4;
        public static final double kIntakePower = 0.75;
        public static final boolean kFrontIntakeInverted = true;
        public static final boolean kBackIntakeInverted = false;

    }

    public static class Climb {

        // Climb Constants
        public static final int kLeftClimbTop = 3;
        public static final int kLeftClimbBottom = 2;
        public static final int kRightClimbTop = 1;
        public static final int kRightClimbBottom = 0;
    
    }

}
