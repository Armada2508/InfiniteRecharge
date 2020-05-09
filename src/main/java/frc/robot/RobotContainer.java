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
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.motion.*;
import frc.robot.commands.*;
import frc.robot.enums.ClimbState;
import frc.robot.subsystems.*;

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
        new POVButton(mJoystick, 90).whileHeld(new DrivePower(mDrive, Constants.Drive.kCreepSpeed, -Constants.Drive.kCreepSpeed));
        new POVButton(mJoystick, 180).whileHeld(new DrivePower(mDrive, -Constants.Drive.kCreepSpeed, -Constants.Drive.kCreepSpeed));
        new POVButton(mJoystick, 270).whileHeld(new DrivePower(mDrive, -Constants.Drive.kCreepSpeed, Constants.Drive.kCreepSpeed));
        new JoystickButton(mButtonBoard, 2).whileHeld(new Intake(mIntake, Constants.Intake.kPower));
        new JoystickButton(mButtonBoard, 1).whileHeld(new Intake(mIntake, -Constants.Intake.kPower));
        new JoystickButton(mButtonBoard, 15).whenHeld(new SpinRoller(mShooter, 6300));
        new JoystickButton(mButtonBoard, 4).whileHeld(new TransportPower(mTransport, 1.0));
        new JoystickButton(mButtonBoard, 12).whileHeld(new Climb(mClimb, ClimbState.EXTENDED));
        new JoystickButton(mButtonBoard, 13).whileHeld(new Climb(mClimb, ClimbState.VENTED));
        new JoystickButton(mButtonBoard, 14).whileHeld(new Climb(mClimb, ClimbState.RETRACTED));
        new JoystickButton(mButtonBoard, 3).whileHeld(new TransportPower(mTransport, -1.0));
        new JoystickButton(mButtonBoard, 6).whileHeld(new Aim(mDrive, mVision));

    }
    

    public void initDashboard() {
        
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
                .addNumber("Current Draw", mDrive.getCurrent()[i])
                .withWidget(BuiltInWidgets.kGraph)
                .withPosition(0, 1);
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
            .addNumber("Turn Velocity", mDrive::getTurnVelocity)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(8, 0)
            .withSize(4, 3);
        Shuffleboard.getTab("Robot")
            .addNumber("Gyro", mDrive::getHeading)
            .withWidget(BuiltInWidgets.kGyro)
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

        
        // ================
        //      Power
        // ================
        Shuffleboard.getTab("Power")
            .add("Power", mPDP)
            .withPosition(0, 0)
            .withSize(6, 5);
        Shuffleboard.getTab("Power")
            .addNumber("Total Energy", mPDP::getTotalEnergy)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(6,0)
            .withSize(6,5);
        
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
        Shuffleboard.getTab("WOF")
            .addNumber("WOF Current", mWOF::getCurrent)
            .withPosition(9, 3)
            .withSize(4, 3)
            .withWidget(BuiltInWidgets.kGraph);


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

        // Configure global parameters for trajectory following
        FollowTrajectory.config(Constants.Drive.kFeedforward.ks, Constants.Drive.kFeedforward.kv, Constants.Drive.kFeedforward.ka, Constants.Drive.kB, Constants.Drive.kZeta, Constants.Drive.kTrackWidth, Constants.Drive.kPathPID);


        // Follow a Trajectory
        return FollowTrajectory.getCommand(mDrive,
            new Pose2d(),
            new Pose2d(2.0, 0.5, new Rotation2d(Math.PI/4.0)),
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
