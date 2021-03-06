/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.motion.*;
import frc.lib.util.LogUtil;
import frc.robot.commands.*;
import frc.robot.enums.ClimbState;
import frc.robot.subsystems.*;

import java.nio.file.Paths;
import java.util.*;

import badlog.lib.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems are defined here
    private final DriveSubsystem mDrive = new DriveSubsystem();
    private final TransportSubsystem mTransport = new TransportSubsystem();
    private final ShooterSubsystem mShooter = new ShooterSubsystem();
    private final IntakeSubsystem mIntake = new IntakeSubsystem(Constants.Intake.kTalon, Constants.Intake.kInverted);
    private final PneumaticsSubsystem mPneumatics = new PneumaticsSubsystem();
    private final ClimbSubsystem mClimb = new ClimbSubsystem();
    private final VisionSubsystem mVision = new VisionSubsystem();
    private final ColorWheelSubsystem mWOF = new ColorWheelSubsystem();
    private final Joystick mJoystick = new Joystick(Constants.Drive.kJoystickPort);
    private final Joystick mButtonBoard = new Joystick(Constants.ButtonBoard.kPort);
    private final PowerDistributionPanel mPDP = new PowerDistributionPanel();
    private BadLog mLogger;
    private Timer mLoggerTimer;
    private double mLastTime = 0.0;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

    }

    public void robotInit() {
        
        // Initialize the Dashboard
        initDashboard();

        // Initialize the Logger
        initLogger();

        // Initialize the Camera(s)
        initCam();
        
        // Reset the total PDP energy measured
        mPDP.resetTotalEnergy();

        // Setup the vision subsystem
        mVision.setup();

        // Tell the Drive Subsystem to drive if its bored
        mDrive.setDefaultCommand(new DriveClosedLoop(mDrive, () -> (mJoystick.getRawAxis(Constants.Drive.kThrottleAxis) * (Constants.Drive.kThrottleInverted ? -1.0 : 1.0)),
            () -> (mJoystick.getRawAxis(Constants.Drive.kTrimAxis) * (Constants.Drive.kTrimInverted ? -1.0 : 1.0)),
            () -> (mJoystick.getRawAxis(Constants.Drive.kTurnAxis) * (Constants.Drive.kTurnInverted ? -1.0 : 1.0))));
        

        mClimb.setDefaultCommand(new Climb(mClimb, ClimbState.VENTED));

        // Tell the Transport Subsystem to do its thing if its bored
        mTransport.setDefaultCommand(new AutoTransport(mTransport, new JoystickButton(mButtonBoard, 16)::get));
    }


    private void configureButtonBindings() {

        // =======================
        //      Map Buttons
        // =======================
        
        new POVButton(mJoystick, 0).whileHeld(new DrivePower(mDrive, Constants.Drive.kCreepSpeed, Constants.Drive.kCreepSpeed));
        new POVButton(mJoystick, 45).whileHeld(new DrivePower(mDrive, 2*Constants.Drive.kCreepSpeed, 0.0));
        new POVButton(mJoystick, 90).whileHeld(new DrivePower(mDrive, Constants.Drive.kCreepSpeed, 0.0));
        new POVButton(mJoystick, 135).whileHeld(new DrivePower(mDrive, -2*Constants.Drive.kCreepSpeed, 0.0));
        new POVButton(mJoystick, 180).whileHeld(new DrivePower(mDrive, -Constants.Drive.kCreepSpeed, -Constants.Drive.kCreepSpeed));
        new POVButton(mJoystick, 225).whileHeld(new DrivePower(mDrive, 0.0, -2*Constants.Drive.kCreepSpeed));
        new POVButton(mJoystick, 270).whileHeld(new DrivePower(mDrive, 0.0, Constants.Drive.kCreepSpeed));
        new POVButton(mJoystick, 315).whileHeld(new DrivePower(mDrive, 0.0, 2*Constants.Drive.kCreepSpeed));
        new JoystickButton(mButtonBoard, 2).whileHeld(new Intake(mIntake, Constants.Intake.kPower));
        new JoystickButton(mButtonBoard, 1).whileHeld(new Intake(mIntake, -Constants.Intake.kPower));
        new JoystickButton(mButtonBoard, 15).whenHeld(new SpinRoller(mShooter, 6300));
        new JoystickButton(mButtonBoard, 4).whileHeld(new TransportPower(mTransport, 1.0));
        new JoystickButton(mButtonBoard, 12).whileHeld(new Climb(mClimb, ClimbState.EXTENDED));
        new JoystickButton(mButtonBoard, 13).whileHeld(new Climb(mClimb, ClimbState.VENTED));
        new JoystickButton(mButtonBoard, 14).whileHeld(new Climb(mClimb, ClimbState.RETRACTED));
        new JoystickButton(mButtonBoard, 3).whileHeld(new TransportPower(mTransport, -1.0));
        new JoystickButton(mButtonBoard, 6).whileHeld(new Aim(mDrive, mVision, Constants.Vision.kAimSamples));

    }
    

    public void initDashboard() {

        // Disable LiveWindow
        LiveWindow.disableAllTelemetry();
        
        // ================
        //      Drive
        // ================

        // Physical Drive Stuff

        NetworkTableEntry lRef = NetworkTableInstance.getDefault().getTable("ramsete").getEntry("left_reference");
        NetworkTableEntry rRef = NetworkTableInstance.getDefault().getTable("ramsete").getEntry("right_reference");

        Shuffleboard.getTab("Drive Physical")
            .getLayout("Left", BuiltInLayouts.kGrid)
            .withPosition(0, 0)
            .withSize(6, 6)
            .withProperties(Map.of("Number of columns", 1))
            .addDoubleArray("Velocity", () -> { return new double[] { mDrive.getVelocityLeft(), lRef.getDouble(0)};})
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(0, 0);
        Shuffleboard.getTab("Drive Physical")
            .getLayout("Left")
            .addNumber("Position", mDrive::getPositionLeft)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(0, 1);
        Shuffleboard.getTab("Drive Physical")
            .getLayout("Right", BuiltInLayouts.kGrid)
            .withPosition(6, 0)
            .withSize(6, 6)
            .withProperties(Map.of("Number of columns", 1))
            .addDoubleArray("Velocity", () -> { return new double[] { mDrive.getVelocityRight(), rRef.getDouble(0)};})
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(0, 0);
        Shuffleboard.getTab("Drive Physical")
            .getLayout("Right")
            .addNumber("Position", mDrive::getPositionRight)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(0, 1);
        // Electrical Drive Stuff
        for (int i = 0; i < mDrive.getIDs().length; i++) {
            Shuffleboard.getTab("Drive Electrical")
                .getLayout("Talon " + mDrive.getIDs()[i], BuiltInLayouts.kGrid)
                .withPosition(0+3*i, 0)
                .withSize(3, 6)
                .withProperties(Map.of("Number of columns", 1))
                .addNumber("Output Voltage", mDrive.getVoltage()[i])
                .withWidget(BuiltInWidgets.kGraph)
                .withPosition(0, 0);
            Shuffleboard.getTab("Drive Electrical")
                .getLayout("Talon " + mDrive.getIDs()[i])
                .addBoolean("Inverted", mDrive.getInverted()[i])
                .withPosition(0, 2);
        }

        // Global Robot Stuff
        Shuffleboard.getTab("Robot")
            .addNumber("Robot Velocity", mDrive::getVelocity)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(4, 0)
            .withSize(4, 3);
        Shuffleboard.getTab("Robot")
            .addNumber("Gyro", mDrive::getHeading)
            .withPosition(8, 3)
            .withSize(4, 3);
        Shuffleboard.getTab("Robot")
            .addNumber("Odometry Heading", mDrive::getOdometryHeading)
            .withPosition(4, 3)
            .withSize(4, 3);
        Shuffleboard.getTab("Robot")
            .addNumber("Odometry X", mDrive::getOdometryX)
            .withPosition(0, 0)
            .withSize(4, 3);
        Shuffleboard.getTab("Robot")
            .addNumber("Odometry Y", mDrive::getOdometryY)
            .withPosition(0, 3)
            .withSize(4, 3);
        Shuffleboard.getTab("Vision")
            .addBoolean("Is Aiming", mDrive::isAiming)
            .withPosition(12, 0)
            .withSize(1, 6);
        
        // =================
        //      Intake
        // =================
        Shuffleboard.getTab("Intake")
            .getLayout("Talon " + mIntake.getID(), BuiltInLayouts.kGrid)
            .withPosition(0, 0)
            .withSize(4, 6)
            .withProperties(Map.of("Number of columns", 1))
            .addNumber("Output Voltage", mIntake::getVoltage)
            .withWidget(BuiltInWidgets.kGraph);
        Shuffleboard.getTab("Intake")
            .getLayout("Talon " + mIntake.getID())
            .addNumber("Current Draw", mIntake::getCurrent)
            .withWidget(BuiltInWidgets.kGraph);
        Shuffleboard.getTab("Intake")
            .getLayout("Talon " + mIntake.getID())
            .addBoolean("Inverted", mIntake::getInverted);
        
        // ================
        //      Climb
        // ================
        
        Shuffleboard.getTab("Climb")
            .getLayout("Climb State", BuiltInLayouts.kGrid)
            .withSize(4, 6)
            .withPosition(0, 0)
            .withProperties(Map.of("Number of columns", 1))
            .addBoolean("Extended", mClimb::isExtended)
            .withPosition(0, 0);
        Shuffleboard.getTab("Climb")
            .getLayout("Climb State")
            .addBoolean("Retracted", mClimb::isRetracted)
            .withPosition(0, 2);
        Shuffleboard.getTab("Climb")
            .getLayout("Climb State")
            .addBoolean("Vented", mClimb::isVented)
            .withPosition(0, 1);

        // ================
        //      Vision 
        // ================

        Shuffleboard.getTab("Vision")
            .addBoolean("Target Found", mVision::targetFound)
            .withPosition(0, 0)
            .withSize(4, 3);
        Shuffleboard.getTab("Vision")
            .addNumber("Target X", mVision::getX)
            .withPosition(8, 0)
            .withSize(4, 3)
            .withWidget(BuiltInWidgets.kGraph);
        Shuffleboard.getTab("Vision")
            .addNumber("Target Y", mVision::getY)
            .withPosition(8, 3)
            .withSize(4, 3)
            .withWidget(BuiltInWidgets.kGraph);
        Shuffleboard.getTab("Vision")
            .addNumber("Target Width", mVision::getTargetWidth)
            .withPosition(4, 0)
            .withSize(4, 3);
        Shuffleboard.getTab("Vision")
            .addNumber("Target Height", mVision::getTargetHeight)
            .withPosition(4, 3)
            .withSize(4, 3);
        Shuffleboard.getTab("Vision")
            .addNumber("Target Angle", mVision::getTargetAngle)
            .withPosition(0, 3)
            .withSize(4, 3);


        // ======================
        //      Color Wheel
        // ======================
        Shuffleboard.getTab("WOF")
            .addNumber("WOF Rotations", mWOF::getRotations)
            .withPosition(0, 0)
            .withSize(5, 3)
            .withWidget(BuiltInWidgets.kGraph);
        Shuffleboard.getTab("WOF")
            .addNumber("WOF RPM", mWOF::getRPM)
            .withPosition(5, 0)
            .withSize(4, 3);
        Shuffleboard.getTab("WOF")
            .addNumber("WOF Voltage", mWOF::getVoltage)
            .withPosition(9, 0)
            .withSize(4, 3)
            .withWidget(BuiltInWidgets.kGraph);
        Shuffleboard.getTab("WOF")
            .addBoolean("WOF Inverted", mWOF::getInverted)
            .withPosition(0, 3)
            .withSize(5, 3);
        Shuffleboard.getTab("WOF")
            .addString("WOF Color", mWOF::getColorString)
            .withPosition(5, 3)
            .withSize(4, 3);


        // =====================
        //      Pneumatics
        // =====================
        Shuffleboard.getTab("Pneumatics")
            .addBoolean("Pressure Switch", mPneumatics::getPressureSwitch)
            .withPosition(0, 0)
            .withSize(3, 6);
        Shuffleboard.getTab("Pneumatics")
            .getLayout("Pneumatics Faults", BuiltInLayouts.kGrid)
            .withSize(5, 6)
            .withPosition(3, 0)
            .withProperties(Map.of("Number of columns", 1))
            .addBoolean("Not Connected Fault", mPneumatics::getCompressorNotConnectedFault)
            .withPosition(0, 0)
            .withSize(6, 2);
        Shuffleboard.getTab("Pneumatics")
            .getLayout("Pneumatics Faults")
            .addBoolean("Current Fault", mPneumatics::getCompressorCurrentTooHighFault)
            .withPosition(0, 1)
            .withSize(6, 2);
        Shuffleboard.getTab("Pneumatics")
            .getLayout("Pneumatics Faults")
            .addBoolean("Short Fault", mPneumatics::getCompressorShortedFault)
            .withPosition(0, 2)
            .withSize(6, 2);

        // =================
        //      Shooter
        // =================

        
        for (int i = 0; i < mShooter.getIDs().length; i++) {
            Shuffleboard.getTab("Shooter")
                .getLayout("Talon " + mShooter.getIDs()[i], BuiltInLayouts.kGrid)
                .withPosition(0+4*i, 0)
                .withSize(4, 7)
                .withProperties(Map.of("Number of columns", 1))
                .addNumber("Output Voltage", mShooter.getVoltage()[i])
                .withWidget(BuiltInWidgets.kGraph)
                .withPosition(0, 0);
            Shuffleboard.getTab("Shooter")
                .getLayout("Talon " + mShooter.getIDs()[i])
                .addNumber("Current Draw", mShooter.getCurrent()[i])
                .withWidget(BuiltInWidgets.kGraph)
                .withPosition(0, 1);
            Shuffleboard.getTab("Shooter")
                .getLayout("Talon " + mShooter.getIDs()[i])
                .addBoolean("Inverted", mShooter.getInverted()[i])
                .withPosition(0, 2);
        }
        Shuffleboard.getTab("Shooter")
            .addNumber("Shooter RPM", mShooter::getRPM)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(8, 0)
            .withSize(5, 4);

    }

    public void initLogger() {
        mLoggerTimer = new Timer();
        mLoggerTimer.start();

        String session = LogUtil.getSessionName();
        mLogger = BadLog.init(Paths.get(Filesystem.getOperatingDirectory().getAbsolutePath(), "log", session + ".badlog").toString());
        
        BadLog.createValue("Start Time", LogUtil.getTimestamp());
        BadLog.createValue("Event Name", Optional.ofNullable(DriverStation.getInstance().getEventName()).orElse(""));
        BadLog.createValue("Match Type", DriverStation.getInstance().getMatchType().toString());
        BadLog.createValue("Match Number", "" + DriverStation.getInstance().getMatchNumber());
        BadLog.createValue("Alliance", DriverStation.getInstance().getAlliance().toString());
        BadLog.createValue("Alliance Station", "" + DriverStation.getInstance().getLocation());
        BadLog.createTopic("Match Time", "s", Timer::getMatchTime, "zero");



        // ================
        //      Drive
        // ================

        // Physical Drive Stuff
        BadLog.createTopic("Ramsete/Left Reference", "m/s", () -> NetworkTableInstance.getDefault().getTable("ramsete").getEntry("left_reference").getDouble(0));
        BadLog.createTopic("Ramsete/Right Reference", "m/s", () -> NetworkTableInstance.getDefault().getTable("ramsete").getEntry("right_reference").getDouble(0));

        BadLog.createTopic("Drive Physical/Left Velocity", "m/s", () -> mDrive.getVelocityLeft());
        BadLog.createTopic("Drive Physical/Left Position", "m", mDrive::getPositionLeft);
        BadLog.createTopic("Drive Physical/Right Velocity", "m/s", () -> mDrive.getVelocityRight());
        BadLog.createTopic("Drive Physical/Right Position", "m", mDrive::getPositionRight);
        
        // Electrical Drive Stuff
        for (int i = 0; i < mDrive.getIDs().length; i++) {
            final int index = i;
            BadLog.createTopic("Drive Electrical/Talon " + mDrive.getIDs()[i] + " Output", "V", () -> mDrive.getVoltage()[index].getAsDouble());
            BadLog.createTopic("Drive Electrical/Talon " + mDrive.getIDs()[i] + " Current", "A", () -> mDrive.getCurrent()[index].getAsDouble());
            BadLog.createTopic("Drive Electrical/Talon " + mDrive.getIDs()[i] + " Temperature", "C", () -> mDrive.getTemps()[index].getAsDouble(), "zero");
        }

        // Global Robot Stuff
        BadLog.createTopic("Robot/Velocity", "m/s", mDrive::getVelocity);
        BadLog.createTopic("Robot/Turn Rate", "rad/s", mDrive::getTurnRate);
        BadLog.createTopic("Robot/Gyro", "deg", mDrive::getHeading);
        BadLog.createTopic("Robot/Odometry Heading", "deg", mDrive::getOdometryHeading);
        BadLog.createTopic("Robot/Odometry X", "m", mDrive::getOdometryX);
        BadLog.createTopic("Robot/Odometry Y", "m", mDrive::getOdometryY);
        BadLog.createTopicStr("Robot/Is Aiming", "bool", () -> LogUtil.boolToString(mDrive.isAiming()), "zero");
        


        // ===============
        //      Power
        // ===============

        BadLog.createTopic("Power/Battery Voltage", "J", RobotController::getBatteryVoltage, "zero");
        BadLog.createTopic("Power/Total Energy", "J", mPDP::getTotalEnergy, "zero");
        for (int i = 0; i < 16; i++) {
            final int index = i;
            BadLog.createTopic("Power/Channel " + i + " Current", "A", () -> mPDP.getCurrent(index));
        }
        BadLog.createTopic("Power/Total Current", "A", mPDP::getTotalCurrent);
        BadLog.createTopic("Power/RIO Voltage", "V", RobotController::getInputVoltage);
        BadLog.createTopic("Power/RIO Current", "A", RobotController::getInputCurrent);
        BadLog.createTopicStr("Power/Is Browned Out", "bool", () -> LogUtil.boolToString(RobotController.isBrownedOut()));
        
        
        // ====================
        //      System
        // ====================

        BadLog.createTopicStr("System/FPGA Active", "bool", () -> LogUtil.boolToString(RobotController.isSysActive()));
        BadLog.createTopic("System/CAN Utilization", "percent", () -> { return RobotController.getCANStatus().percentBusUtilization * 100.0; }, "zero");
        BadLog.createTopic("System/Uptime", "s", Timer::getFPGATimestamp, "xaxis", "zero");
        BadLog.createTopicSubscriber("System/Loop Time", "s", DataInferMode.DEFAULT, "zero");
        BadLog.createTopicStr("System/Mode", BadLog.UNITLESS, () -> {
            if(DriverStation.getInstance().isEStopped()) {
                return "-1";
            }
            if(DriverStation.getInstance().isDisabled()) {
                return "0";
            } else {
                if(DriverStation.getInstance().isAutonomous()) {
                    return "1";
                } else if(DriverStation.getInstance().isOperatorControl()) {
                    return "2";
                } else if(DriverStation.getInstance().isTest()) {
                    return "3";
                } else {
                    return "-2";
                }
            }
        });
        

        // =================
        //      Intake
        // =================

        BadLog.createTopic("Intake/Output", "V", mIntake::getVoltage);
        BadLog.createTopic("Intake/Current", "A", mIntake::getCurrent);


        // ================
        //      Climb
        // ================
        
        BadLog.createTopicStr("Climb/State", BadLog.UNITLESS, () -> {
            if(mClimb.isExtended()) {
                return "0";
            } else if(mClimb.isVented()) {
                return "1";
            } else if(mClimb.isRetracted()) {
                return "2";
            } else {
                return "-1";
            }
        });



        // ================
        //      Vision 
        // ================

        BadLog.createTopicStr("Vision/Target Found", "bool", () -> LogUtil.boolToString(mVision.targetFound()));
        BadLog.createTopic("Vision/Target X", "deg", mVision::getX);
        BadLog.createTopic("Vision/Target Y", "deg", mVision::getY);
        BadLog.createTopic("Vision/Target Width", "px", mVision::getTargetWidth);
        BadLog.createTopic("Vision/Target Height", "px", mVision::getTargetHeight);
        BadLog.createTopic("Vision/Target Angle", "deg", mVision::getTargetAngle);
        BadLog.createTopic("Vision/Target Distance", "m", () -> mVision.getDistanceHeight(Constants.Vision.kTargetHeight), "zero");



        // ======================
        //      Color Wheel
        // ======================
        BadLog.createTopic("WOF/Rotations", BadLog.UNITLESS, mWOF::getRotations);
        BadLog.createTopic("WOF/RPM", "rpm", mWOF::getRPM);
        BadLog.createTopic("WOF/Output", "V", mWOF::getVoltage);
        BadLog.createTopic("WOF/Current", "A", mWOF::getCurrent);
        BadLog.createTopicStr("WOF/Color", BadLog.UNITLESS, () -> {
            if(!mWOF.isColor()) {
                return "0";
            }

            char c = mWOF.getColor();
            if (c == 'R') {
                return "1";
            } if (c == 'Y') {
                return "2";
            } else if (c == 'G') {
                return "3";
            } else if (c == 'B') {
                return "4";
            } else {
                return "-1";
            }
        });



        // =====================
        //      Pneumatics
        // =====================
        BadLog.createTopicStr("Pneumatics/Pressure Switch", "bool", () -> LogUtil.boolToString(mPneumatics.getPressureSwitch()));
        BadLog.createTopicStr("Pneumatics/Compressor Not Connected", "bool", () -> LogUtil.boolToString(mPneumatics.getCompressorNotConnectedFault()));
        BadLog.createTopicStr("Pneumatics/Compresor Current Too High", "bool", () -> LogUtil.boolToString(mPneumatics.getCompressorCurrentTooHighFault()));
        BadLog.createTopicStr("Pneumatics/Compressor Shorted", "bool", () -> LogUtil.boolToString(mPneumatics.getCompressorShortedFault()));

        // =================
        //      Shooter
        // =================

        
        BadLog.createTopic("Shooter/RPM", "rpm", mShooter::getRPM);
        for (int i = 0; i < mShooter.getIDs().length; i++) {
            final int index = i;
            BadLog.createTopic("Shooter/Talon " + mShooter.getIDs()[i] + " Output", "V", () -> mShooter.getVoltage()[index].getAsDouble());
            BadLog.createTopic("Shooter/Talon " + mShooter.getIDs()[i] + " Current", "A", () -> mShooter.getCurrent()[index].getAsDouble());
            BadLog.createTopic("Shooter/Talon " + mShooter.getIDs()[i] + " Temperature", "C", () -> mShooter.getTemp()[index].getAsDouble(), "zero");
        }
        mLogger.finishInitialization();
        
    }


    public void initCam() {
        // Get the back camera plugged into the RIO
        UsbCamera backCamera = CameraServer.getInstance().startAutomaticCapture(0);
        // Feed that back camera into a new stream so we can add compression
        MjpegServer backCameraStream = CameraServer.getInstance().startAutomaticCapture(backCamera);
        // Compress the stream, set its resolution, and set its framerate along with the camera's
        backCameraStream.setCompression(Constants.Camera.kCameraCompression);
        backCamera.setResolution(Constants.Camera.kCameraResolution.getX(), Constants.Camera.kCameraResolution.getY());
        backCameraStream.setResolution(Constants.Camera.kCameraResolution.getX(), Constants.Camera.kCameraResolution.getY());
        backCamera.setFPS(Constants.Camera.kCameraFPS);
        backCameraStream.setFPS(Constants.Camera.kCameraFPS);
        
    }

    public void updateLogger() {
        if(mLoggerTimer.hasPeriodPassed(DriverStation.getInstance().isEnabled() ? Constants.Logging.kEnabledLogPeriod / 1000.0 : Constants.Logging.kDisabledLogPeriod / 1000.0)) {
            BadLog.publish("System/Loop Time", Timer.getFPGATimestamp() - mLastTime);

            mLogger.updateTopics();

            mLogger.log();
        }
        mLastTime = Timer.getFPGATimestamp();
    }
    
    public void changeMode() {
        // Reset the Vision Subsystem because robotInit doesn't work when connected to the FMS for some reason
        mVision.reset();
        mDrive.reset(true);
    }

    public void stopTalons() {
        // Stop powering the talons
        mDrive.setVoltage(0.0, 0.0);
    }

    public Command getAutonomousCommand() {

        mDrive.reset(false);

        // Configure global parameters for trajectory following
        FollowTrajectory.config(Constants.Drive.kFeedforward.ks, Constants.Drive.kFeedforward.kv, Constants.Drive.kFeedforward.ka, Constants.Drive.kB, Constants.Drive.kZeta, Constants.Drive.kTrackWidth, Constants.Drive.kPathPID, Constants.Drive.kTurnCompensation);


        // Follow a Trajectory
        return FollowTrajectory.getCommandTalon(mDrive,
            new Pose2d(),
            new Pose2d(2.0, 0.0, new Rotation2d(0.0)),
            Constants.Drive.kMaxVelocity,
            Constants.Drive.kMaxAcceleration,
            Constants.Drive.kMaxVoltage,
            Constants.Drive.kMaxCentripetalAcceleration,
            false);
            
    }

    public void printOdo() {
        // Print the odometer
        System.out.println(mDrive.getPose());
    }

    public void printPos() {
        // Print both wheel encoder positions
        System.out.println(new DifferentialDriveWheelPositions(mDrive.getPositionLeft(), mDrive.getPositionRight()));
    }

    public void printVel() {
        // Print the wheel velocities
        System.out.println(mDrive.getWheelSpeeds());
    }
    
}
