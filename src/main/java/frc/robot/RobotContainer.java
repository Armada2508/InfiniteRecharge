/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint.MinMax;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.logging.ShuffleboardManager;
import frc.lib.motion.*;
import frc.robot.commands.*;
import frc.robot.enums.ClimbState;
import frc.robot.routines.MoveForward;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.*;

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
    private final ShuffleboardTab mShooterTable = Shuffleboard.getTab("Shooter");
    private final PowerDistributionPanel mPDP = new PowerDistributionPanel();

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

        // Initialize the Camera(s)
        initCam();

        // Setup the vision subsystem
        mVision.setup();

        // Tell the Drive Subsystem to drive if its bored
        /*mDrive.setDefaultCommand(new DriveClosedLoop(mDrive, () -> (mJoystick.getRawAxis(Constants.Drive.kThrottleAxis) * (Constants.Drive.kThrottleInverted ? -1.0 : 1.0)),
            () -> (mJoystick.getRawAxis(Constants.Drive.kTrimAxis) * (Constants.Drive.kTrimInverted ? -1.0 : 1.0)),
            () -> (mJoystick.getRawAxis(Constants.Drive.kTurnAxis) * (Constants.Drive.kTurnInverted ? -1.0 : 1.0))));
        */

        // Tell the Transport Subsystem to do its thing if its bored
        mTransport.setDefaultCommand(new AutoTransport(mTransport, new JoystickButton(mButtonBoard, 16)::get));
    }


    private void configureButtonBindings() {

        // =======================
        //      Map Buttons
        // =======================
        
        //new JoystickButton(mJoystick, 3).whenHeld(new Intake(mIntake,  Constants.Intake.kIntakePower));
        //new JoystickButton(mJoystick, 1).whileHeld(new TransportPower(mTransport, 1.0));
        //new JoystickButton(mJoystick, 2).whenHeld(new SpinRoller(mShooter, 2000));
        //new POVButton(mJoystick, 0).whileHeld(new DrivePower(mDrive, Constants.Drive.kCreepSpeed, Constants.Drive.kCreepSpeed));
        //new POVButton(mJoystick, 90).whileHeld(new DrivePower(mDrive, Constants.Drive.kCreepSpeed, -Constants.Drive.kCreepSpeed));
        //new POVButton(mJoystick, 180).whileHeld(new DrivePower(mDrive, -Constants.Drive.kCreepSpeed, -Constants.Drive.kCreepSpeed));
        //new POVButton(mJoystick, 270).whileHeld(new DrivePower(mDrive, -Constants.Drive.kCreepSpeed, Constants.Drive.kCreepSpeed));
        new JoystickButton(mButtonBoard, 2).whileHeld(new Intake(mIntake, Constants.Intake.kPower));
        new JoystickButton(mButtonBoard, 1).whileHeld(new Intake(mIntake, -Constants.Intake.kPower));
        new JoystickButton(mButtonBoard, 15).whenHeld(new SpinRoller(mShooter, 2000));
        new JoystickButton(mButtonBoard, 4).whileHeld(new TransportPower(mTransport, 1.0));
        new JoystickButton(mButtonBoard, 12).whenHeld(new Climb(mClimb, ClimbState.EXTENDED));
        new JoystickButton(mButtonBoard, 13).whenHeld(new Climb(mClimb, ClimbState.VENTED));
        new JoystickButton(mButtonBoard, 14).whenHeld(new Climb(mClimb, ClimbState.RETRACTED));
        new JoystickButton(mButtonBoard, 3).whileHeld(new TransportPower(mTransport, -1.0));
        new JoystickButton(mButtonBoard, 6).whileHeld(new Aim(mDrive, mVision));

    }
    

    public void initDashboard() {
        
        // ================
        //      Drive
        // ================

        // Talon Stuff
        for (int i = 0; i < mDrive.getVoltage().length; i++) {
            ShuffleboardManager.logDouble(mDrive.getVoltage()[i], "Talon " + mDrive.getIDs()[i] + " Output Voltage"); 
        }
        for (int i = 0; i < mDrive.getCurrent().length; i++) {
            ShuffleboardManager.logDouble(mDrive.getCurrent()[i], "Talon " + mDrive.getIDs()[i] + " Current Draw"); 
        }
        for (int i = 0; i < mDrive.getTemps().length; i++) {
            ShuffleboardManager.logDouble(mDrive.getTemps()[i], "Talon " + mDrive.getIDs()[i] + " Temperature"); 
        }
        for (int i = 0; i < mDrive.getVelocities().length; i++) {
            ShuffleboardManager.logDouble(mDrive.getVelocities()[i], "Talon " + mDrive.getIDs()[i] + " Velocity"); 
        }
        for (int i = 0; i < mDrive.getPositions().length; i++) {
            ShuffleboardManager.logDouble(mDrive.getPositions()[i], "Talon " + mDrive.getIDs()[i] + " Position"); 
        }
        for (int i = 0; i < mDrive.getInverted().length; i++) {
            ShuffleboardManager.logBoolean(mDrive.getInverted()[i], "Talon " + mDrive.getIDs()[i] + " Inverted"); 
        }
        // Other Drive Stuff
        ShuffleboardManager.logDouble(mDrive::getVelocity, "Robot Velocity");
        ShuffleboardManager.logDouble(mDrive::getTurnVelocity, "Turn Velocity");
        ShuffleboardManager.logDouble(mDrive::getUnboundedHeading, "Gyro");
        ShuffleboardManager.logDouble(mDrive::getOdometryHeading, "Odometry Heading");
        ShuffleboardManager.logDouble(mDrive::getOdometryX, "Odometry X");
        ShuffleboardManager.logDouble(mDrive::getOdometryY, "Odometry Y");
        ShuffleboardManager.logBoolean(mDrive::isAiming, "Is Aiming");

        
        // ================
        //      Power
        // ================
        ShuffleboardManager.logPDP(mPDP);
        ShuffleboardManager.logDouble(mPDP::getTotalEnergy, "Total Energy");
        
        // =================
        //      Intake
        // =================
        ShuffleboardManager.logDouble(mIntake::getVoltage, "Talon " + mIntake.getID() + " Voltage");
        ShuffleboardManager.logDouble(mIntake::getCurrent, "Talon " + mIntake.getID() + " Current");
        ShuffleboardManager.logDouble(mIntake::getTemp, "Talon " + mIntake.getID() + " Temperature");
        
        
        // ================
        //      Climb
        // ================
        ShuffleboardManager.logBoolean(mClimb::isExtended, "Extended");
        ShuffleboardManager.logBoolean(mClimb::isRetracted, "Retracted");
        ShuffleboardManager.logBoolean(mClimb::isVented, "Vented");

        // ================
        //      Vision 
        // ================

        ShuffleboardManager.logBoolean(mVision::targetFound, "Found Target");
        ShuffleboardManager.logDouble(mVision::getX, "Target X");
        ShuffleboardManager.logDouble(mVision::getY, "Target Y");
        ShuffleboardManager.logDouble(mVision::getTargetWidth, "Target Width");
        ShuffleboardManager.logDouble(mVision::getTargetHeight, "Target Height");
        ShuffleboardManager.logDouble(mVision::getTargetAngle, "Target Angle");
        ShuffleboardManager.logDouble(mVision::getTargetHeight, "Target Height");


        // ======================
        //      Color Wheel
        // ======================
        ShuffleboardManager.logDouble(mWOF::getRotations, "WOF Rotations");
        ShuffleboardManager.logDouble(mWOF::getRPM, "WOF RPM");
        ShuffleboardManager.logString(mWOF::getColorString, "WOF Color");
        ShuffleboardManager.logDouble(mWOF::getVoltage, "WOF Voltage");
        ShuffleboardManager.logDouble(mWOF::getCurrent, "WOF Current");
        ShuffleboardManager.logDouble(mWOF::getTemp, "WOF Temperature");
        ShuffleboardManager.logBoolean(mWOF::getInverted, "WOF Inverted");


        // =====================
        //      Pneumatics
        // =====================
        ShuffleboardManager.logBoolean(mPneumatics::getPressureSwitch, "Pressure Switch");
        ShuffleboardManager.logBoolean(mPneumatics::getCompressorCurrentTooHighFault, "Current Fault");
        ShuffleboardManager.logBoolean(mPneumatics::getCompressorCurrentTooHighStickyFault, "Current Sticky Fault");
        ShuffleboardManager.logBoolean(mPneumatics::getCompressorShortedFault, "Short Fault");
        ShuffleboardManager.logBoolean(mPneumatics::getCompressorShortedStickyFault, "Short Sticky Fault");
        ShuffleboardManager.logBoolean(mPneumatics::getCompressorNotConnectedFault, "Not Connected Fault");
        ShuffleboardManager.logBoolean(mPneumatics::getCompressorNotConnectedStickyFault, "Not Connected Sticky Fault");

        // =================
        //      Shooter
        // =================
        ShuffleboardManager.logDouble(mShooter::getRPM, "Shooter RPM");
        for (int i = 0; i < mShooter.getVoltage().length; i++) {
            ShuffleboardManager.logDouble(mShooter.getVoltage()[i], "Talon " + mShooter.getIDs()[i] + " Output Voltage"); 
        }
        for (int i = 0; i < mShooter.getCurrent().length; i++) {
            ShuffleboardManager.logDouble(mShooter.getCurrent()[i], "Talon " + mShooter.getIDs()[i] + " Current Draw"); 
        }
        for (int i = 0; i < mShooter.getTemp().length; i++) {
            ShuffleboardManager.logDouble(mShooter.getTemp()[i], "Talon " + mShooter.getIDs()[i] + " Temperature"); 
        }

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

    public void startDashboardCapture() {
        // If the FMS is connected, start recording all data sent to shuffleboard
        if(DriverStation.getInstance().isFMSAttached()) {
            Shuffleboard.startRecording();
        }
    }

    public void stopDashboardCapture() {
        // Stop recording
        Shuffleboard.startRecording();
    }

    public void changeMode() {
        // Reset the Vision Subsystem because robotInit doesn't work when connected to the FMS for some reason
        mVision.reset();
        mDrive.reset();
    }

    public void stopTalons() {
        // Stop powering the talons
        mDrive.setVoltage(0.0, 0.0);
    }

    public Command getAutonomousCommand() {


  /*      return FollowTrajectory.getCommand(mDrive,
            new Pose2d(),
            new Pose2d(5, 0, new Rotation2d()),
            Constants.Drive.kMaxVelocity,
            Constants.Drive.kMaxAcceleration);
*/  
        //return new SimpleAuto(mDrive, mShooter, mTransport, mIntake, mVision);
        //return new Aim(mDrive, mVision); FollowTrajectory.config(Constants.Drive.kDriveFeedforward.ks, Constants.Drive.kDriveFeedforward.kv, Constants.Drive.kDriveFeedforward.ka, Constants.Drive.kB, Constants.Drive.kZeta, Constants.Drive.kTrackWidth, Constants.Drive.kPathPID);
        
        // Configure global parameters for trajectory following
        FollowTrajectory.config(Constants.Drive.kFeedforward.ks, Constants.Drive.kFeedforward.kv, Constants.Drive.kFeedforward.ka, Constants.Drive.kB, Constants.Drive.kZeta, Constants.Drive.kTrackWidth, Constants.Drive.kPathPID);

        try {
            Trajectory line = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Line.wpilib.json"));
            Trajectory intake1 = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Intake1.wpilib.json"));
            Trajectory intake2 = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Intake2.wpilib.json"));
            Trajectory turn = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/90Turn.wpilib.json"));
            Trajectory[] paths = { intake1, intake2 };
            return new MoveForward(mDrive, turn);
        } catch (IOException e) {
            // If the trajectory file can't be read, print the error and return a new command that does nothing
            System.out.println(e);
            return new InstantCommand();
        }
    
    }

    public void printOdo() {
        // Print the odometer
        System.out.println(mDrive.getPose());
    }

    public void printPos() {
        // Print both wheel encoder positions
        System.out.println(new WheelPositions(mDrive.getPositionLeft(), mDrive.getPositionRight()));
    }

    public void printVel() {
        // Print the wheel velocities
        System.out.println(mDrive.getWheelSpeeds());
    }
}
