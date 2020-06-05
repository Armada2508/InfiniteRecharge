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
import frc.lib.config.FeedbackConfig;
import frc.lib.config.FeedbackConstants;
import frc.lib.config.MotionMagicConfig;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.lib.vision.FOV;
import frc.lib.vision.Resolution;

public final class Constants {

    // =============================
    //    Button Board Constants
    // =============================
    public static final class ButtonBoard {
        // Button Mapping
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

    // ===============================
    //    Global Robot Constants
    // ===============================
    public static final class Robot {

        public static final double kMinBatteryVoltage = 9; // The lowest voltage the battery is predicted to be

    }


    // =========================
    //    Logging Constants
    // =========================
    public static final class Logging {

        // Logging Periods in ms
        public static final double kDisabledLogPeriod = 250; // The logging period when disabled
        public static final double kEnabledLogPeriod = 20; // The logging period when enabled

    }


    // ======================
    //    Drive Constants
    // ======================
    public static final class Drive {

        // Motor Ports
        public static final int kLeftMotorPort = 0; // The left drive motor port
        public static final int kLeftMotorFollowerPort = 1; // The left follower drive motor port
        public static final int kRightMotorPort = 2; // The right drive motor port
        public static final int kRightMotorFollowerPort = 3; // The right follower drive motor port

        // Drive Input Constants
        public static final double kDeadbandThreshold = 0.035; // The threshold for sensing input on the joystick
        public static final int kJoystickPort = 0; // The port that the joystick is on
        public static final int kThrottleAxis = 1; // The throttle axis on the joystick
        public static final int kTurnAxis = 2; // The turn axis on the joystick
        public static final int kTrimAxis = 0; // The trim axis on the joystick
        public static final double kMaxPower = 1.0; // The maximum power(0.0-1.0) that the motors will output
        public static final double kTurnRatio = 0.25; // How much the robot likes to turn
        public static final double kTrimRatio = 0.5; // How much the robot trims
        public static final boolean kThrottleInverted = true; // If the throttle axis is inverted
        public static final boolean kTurnInverted = false; // If the turn axis is inverted
        public static final boolean kTrimInverted = false; // If the trim axis is inverted
        public static final double kCreepSpeed = 0.075; // How fast the robot creeps(0.0-1.0)

        // Drive System Constants
        public static final boolean kRightInverted = true; // If the right side is inverted
        public static final boolean kLeftInverted = false; // If the left side is inverted
        public static final int kSlot = 0; // The PID slot to use for the drive subsystem
        public static final FeedbackConstants kFeedbackConstants = new FeedbackConstants(0.125, 0, 0.0, 0.0575, 200); // The feedback constants for the drive
        public static final FeedbackConfig kFeedbackConfig = new FeedbackConfig(FeedbackDevice.IntegratedSensor, 2048, 10.71*(48.0/42.0)); //  The feedback config for the drive
        public static final MotorConfig kConfig = new MotorConfig(40, 0, 0, 0.001);  // The config for the drive TODO: Tune PID
        public static final double kRamp = 0.75; // How fast the Drive will go from 0-100% power

        // Trajectory Following Constants
        public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.31, 1.95, 0.35); // The feedforward for the drive
        public static final PIDController kPathPID = new PIDController(0.25, 0.0,  0); // The PID for trajectory following
        public static final double kTrackWidth = 0.6084; // The track width of the drive
        public static final double kB = 2.0; // The B constant for RamseteController
        public static final double kZeta = 0.7; // The Zeta constant for RamseteController
        public static final double kWheelDiameter = Units.inchesToMeters(8); // The diameter of the wheels on the robot
        public static final double kTurnCompensation = 0.875;  // How much the drive overcompensates for turning because of weird drive dynamics

        // Trajectory Generation Constants
        public static final double kMaxVelocity = 1.0;  // The maximum velocity of the path generated in m/s   // TODO: Determine correct value
        public static final double kMaxAcceleration = 0.75; //The maximum acceleration of the path generated in m/s^2   // TODO: Determine correct value
        public static final double kMaxCentripetalAcceleration = 2.0;  // The maximum centripetal acceleration of the path generated in m/s^2   // TODO: Determine correct value
        public static final double kMaxVoltage = 8.0; //The maximum voltage applied to either side of the robot   // TODO: Determine correct value

    }
    

    // ========================================
    //    Global Motor Controller Constants
    // ========================================
    public static final class MotorController {

        // Status frames are sent over CAN that contain data about the Talon.
        // They are broken up into different pieces of data and the frequency
        // at which they are sent can be changed according to your needs.
        // The period at which their are sent is measured in ms

        public static final int kTalonFrame1Period = 20;  // How often the Talon reports basic info(Limits, limit overrides, faults, control mode, invert)
        public static final int kTalonFrame2Period = 20;  // How often the Talon reports sensor info(Sensor position/velocity, current, sticky faults, profile)
        public static final int kTalonFrame3Period = 160;  // How often the Talon reports non selected quad info(Position/velocity, edges, quad a and b pin, index pin)
        public static final int kTalonFrame4Period = 160;  // How often the Talon reports additional info(Analog position/velocity, temperature, battery voltage, selected feedback sensor)
        public static final int kTalonFrame8Period = 160;  // How often the Talon reports more encoder info(Talon Idx pin, PulseWidthEncoded sensor velocity/position)
        public static final int kTalonFrame10Period = 160;  // How often the Talon reports info on motion magic(Target position, velocity, active trajectory point)
        public static final int kTalonFrame13Period = 160; // How often the Talon reports info on PID(Error, Integral, Derivative)

    }

    // =====================
    //    Gyro Constants
    // =====================

    public static final class Gyro {

        // Status frames are used to transmit data from the gyro to the RIO over
        // CAN.  The frequency at which they are sent can be changed according
        // to you needs to increase performance or decrease utilization.
        // The period at which their are sent is measured in ms

        public static final boolean kGyroReversed = false; // If the gyro is inverted
        public static final int kPigeonCondFrame1Period = 160; // How often the Pigeon reports calibration status and temp
        public static final int kPigeonCondFrame9Period = 10; // Six degree fused Yaw, Pitch, Roll
        public static final int kPigeonCondFrame6Period = 10; // Nine degree fused Yaw, Pitch, Roll
        public static final int kPigeonCondFrame11Period = 20; // Accumulated Gyro Angles
        public static final int kPigeonCondFrame3Period = 160; // Accelerometer derived angles
        public static final int kPigeonCondFrame10Period = 160; // Six degree fused Quaternion
        public static final int kPigeonRawFrame4Period = 160; // Unprocessed magnetometer values (x,y,z)
        public static final int kPigeonBiasedFrame2Period = 160; // Biased gyro values (x,y,z)
        public static final int kPigeonBiasedFrame6Period = 160; // Biased accelerometer values (x,y,z)

    }

    // ==========================
    //    Dashboard Constants
    // ==========================
    public static final class Dashboard {

        // Dashboard Constants
        public static final double kUpdateRate = 0.5; // The dashboard update rate // TODO: Determine correct value

    }

    
    // =======================
    //    Vision Constants
    // =======================
    public static final class Vision {

        public static final FOV kLimelightFOV = new FOV(59.6, 45.7); // The Field-of-view of the limelight
        public static final Resolution kLimelightResolution = new Resolution(960, 720); // The resolution of the limelight
        public static final double kLimelightAngle = 16.5; // The vertical angle the limelight is at //TODO: Determine correct value
        public static final double kLimelightHeight = .794; // The height of the limelight off of the ground //TODO: Determine final height
        public static final double kTargetHeight = 2.5; // The height of the target in meters
        public static final double kVerticalOffset = kTargetHeight - kLimelightHeight; // The difference between the height of the limelight and the height of the target
        public static final double kTargetWidth = 1.0; // The width of the retro-reflective tape on the port
        public static final double kTapeWidth = (4.0 / Math.sqrt(3)) * 0.0254; // The width of the tape 
        public static final double kPAim = 0.02; // The P value used for aiming
        public static final double kIAim = 0.0;  // The I value used for aiming
        public static final double kDAim = 0.0015; // The D value used for aiming
        public static final double kAimOffset = 0.0; // A horizontal offset to use for aiming
        public static final double kMaxAimPower = 0.125; // The maximum motor power used for aiming(0.0-1.0)
        public static final int kAimSamples = 30; // The number of samles used while aiming

    }

    // ======================
    //    Camera Constants
    // ======================
    public static final class Camera {
         
        // Camera Constants
        public static final Resolution kCameraResolution = new Resolution(160, 120); // The resolution to use for streaming the camera to the dashboard
        public static final int kCameraFPS = 14;  // The FPS to use when streaming the camera to the dashboard
        public static final int kCameraCompression = 75; // The compression to use when streaming the camera to the dashboard
    }   

    // =======================
    //    Shooter Constants
    // =======================
    public static final class Shooter {
        
        public static final int kLeftMotor = 11; // The ID of the Shooter Falcon on the left
        public static final int kRightMotor = 10; // The ID of the Shooter Falcon on the right
        public static final FeedbackConstants kFeedbackConstants = new FeedbackConstants(0.05, 0.0, 0.0, 0.0495, 0); // The feedback constants to use for the shooter
        public static final FeedbackConfig kFeedbackConfig = new FeedbackConfig(FeedbackDevice.IntegratedSensor, 2048, 1.0); // The feedback config for the shooter
        public static final MotorConfig kConfig = new MotorConfig(40, 0, 0, 0.001); // The config for the shooter motors // TODO: Redo current on shooter, add voltage compensation
        public static final double kStableRPMThreshold = 100; // The threshold to consider the rpm "Stable" at and switch to a lower current
        public static final double kStableCurrentLimit = 10; // The current limit once the Talon is within the stable threshold
        public static final int kMaxSlewRate = 4000; // The max slew rate in RPM per second to ramp the shooter up at // TODO: Determine correct value
        public static final double kRamp = 0.25; // How fast the motor will ramp from 0% to 100%(in seconds)
        public static final int kSlot = 0; // the PID slot to use for the shooter
        public static final boolean kLeftInverted = true; // If the left shooter motor is inverted
        public static final boolean kRightInverted = false; // If the right shooter motor is inverted
    
    }

    // ===========================
    //    Color Wheel Constants   
    // ===========================
    public static final class WOF {

        public static final double kDiameter = 32; // The diameter of the Wheel-of-Fortune
        public static final double kWheelDiameter = 7.5; // The diameter of the wheel used to turn the wheel of fortune
        public static final FeedbackConstants kFeedbackConstants = new FeedbackConstants(0.01, 0, 0.01, 0.008, 0); // The feedback constants for the Wheel-of-Fortune
        public static final FeedbackConfig kFeedbackConfig = new FeedbackConfig(FeedbackDevice.QuadEncoder, 4096, 20.0); // The feedback config for the Wheel-of-Fortune
        public static final MotorConfig kConfig = new MotorConfig(10, 20, 500, 0.001); // The motor config for the Wheel-of-Fortune
        public static final MotionMagicConfig kMMConfig = new MotionMagicConfig(
            (int)EncoderUtil.fromRPM(300,
                kFeedbackConfig.getEpr(),
                kFeedbackConfig.getGearRatio()),
            (int)EncoderUtil.fromRPM(600,
                kFeedbackConfig.getEpr(),
                kFeedbackConfig.getGearRatio()),
            0); // The Motion Magic config for the Wheel-of-Fortune
        public static final int kSlot = 0; // The PID slot used for the Wheel-of-Fortune talon
        public static final int kTalon = 5; // The ID of the Talon used for the Wheel-of-Fortune
        public static final double kThreshold = 0.02; // How close the Wheel-of-Fortune has to be to end the Motion Magic command
        public static final double kRotations = 3.5; // How far to rotation the Wheel-of-Fortune for rotation control

    }

    // ==========================
    //    Transport Constants
    // ==========================
    public static final class Transport {

        public static final int kTalon = 6; // The ID of the Talon used for the Transport
        public static final double kPulleyDiameter = 2.75; // The diameter of the pulley used in the transport in inches //TODO: Get correct diameter
        public static final FeedbackConstants kFeedbackConstants = new FeedbackConstants(0.15, 0, 0.05, 0, 0); // The feedback constants for the transport in position control mode
        public static final FeedbackConstants kVelocityFeedbackConstants = new FeedbackConstants(0.01, 0, 1.0, 0.030536, 0); // The feedback constants for the transport in velocity control mode
        public static final FeedbackConfig kFeedbackConfig = new FeedbackConfig(FeedbackDevice.QuadEncoder, 4096, 20.0); // The feedback config for the transport
        public static final MotorConfig kConfig = new MotorConfig(15, 25, 500, 0.001); // The motor config for the transport
        public static final MotionMagicConfig kMMConfig = new MotionMagicConfig(
            (int)EncoderUtil.fromVelocity(40.0,
                kFeedbackConfig.getEpr(),
                kFeedbackConfig.getGearRatio(),
                kPulleyDiameter),
            (int)EncoderUtil.fromVelocity(160.0,
                kFeedbackConfig.getEpr(),
                kFeedbackConfig.getGearRatio(),
                kPulleyDiameter),
            0); // The Motion Magic profile for the transport
        public static final int kSlot = 0; // The PID slot to use for the transport
        public static final int kVelocitySlot = 1; // The PID slot to use for velocity control mode
        public static final double kVelocity = 25; // The speed of the transport in inches per second
        public static final double kThreshold = 0.125; // How close the transport has to be to its setpoint to end the motion magic command
        public static final double kMargin = 5.0; // How much to increment the transport after it stops sensing a 
        public static final double kRamp = 0.15; // How fast the motor will ramp from 0% to 100%(in seconds)

        // TOF Sensors
        public static final int kIntakeTOF = 1; // The ID of the intake Time-of-Flight sensor
        public static final int kShooterTOF = 0;  // The ID of the shooter Time-of-Flight sensor
        public static final double kMaxDistance = 130.0;  // The distance to detect a ball at
        public static final double kMaxDeviation = 5.0; // The max deviation allowed for a ball detection
        public static final RangingMode kRangingMode = RangingMode.Short; // Which ranging mode to use
        public static final double kTOFSampleTime = 20; // How often to check for balls(in ms)
        public static final int kDebounceSize = 5; // How much to debounce the Time-of-Flight sensors

    }


    // =======================
    //    Intake Constants
    // =======================
    public static final class Intake {
        public static final int kTalon = 4; // The ID of the Intake Talon
        public static final double kPower = 0.75; // The power to run the Intake at
        public static final boolean kInverted = false; // If the intake is inverted
        public static final double kRamp = 0.25; // How fast the Intake will go from 0-100% power
    }

    // =====================
    //    Climb Constants
    // =====================
    public static final class Climb {

        // Solenoids
        public static final int kLeftTop = 3; // The Solenoid Connected to the top port on the left climb cylinder
        public static final int kLeftBottom = 2; // The Solenoid Connected to the bottom port on the left climb cylinder
        public static final int kRightTop = 1; // The Solenoid Connected to the top port on the right climb cylinder
        public static final int kRightBottom = 0; // The Solenoid Connected to the bottom port on the right climb cylinder
    
    }

}
