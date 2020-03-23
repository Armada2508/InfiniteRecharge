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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem mDrive = new DriveSubsystem();
    private final TransportSubsystem mTransport = new TransportSubsystem();
    private final ShooterSubsystem mShooter = new ShooterSubsystem();
    private final IntakeSubsystem mIntake = new IntakeSubsystem(Constants.Intake.kIntakeTalon, Constants.Intake.kIntakeInverted);
    private final ClimbSubsystem mClimb = new ClimbSubsystem();
    private final VisionSubsystem mVision = new VisionSubsystem();
    private final ColorWheelSubsystem mWOF = new ColorWheelSubsystem();
    private ArrayList<NetworkTableEntry> talonEntries = new ArrayList<NetworkTableEntry>();
    private Joystick mJoystick = new Joystick(Constants.Drive.kJoystickPort);
    private Joystick mButtonBoard = new Joystick(Constants.ButtonBoard.kPort);
    private NetworkTableEntry mGyroEntry;
    private NetworkTableEntry mOdometer;
    private NetworkTableEntry mRPM;
    private ShuffleboardTab mShooterTable = Shuffleboard.getTab("Shooter");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        new PneumaticsSubsystem();
        mRPM = mShooterTable.add("RPM", 0).getEntry();
    }

    public void robotInit() {
        initDashboard();
        mShooter.leftInverted(Constants.Shooter.kShooterLeftInveted);
        mShooter.rightInverted(Constants.Shooter.kShooterRightInverted);
        initCam();
        mVision.setup();
        /*mDrive.setDefaultCommand(new DriveClosedLoop(mDrive, () -> (mJoystick.getRawAxis(Constants.Drive.kThrottleAxis) * (Constants.Drive.kThrottleInverted ? -1.0 : 1.0)),
            () -> (mJoystick.getRawAxis(Constants.Drive.kTrimAxis) * (Constants.Drive.kTrimInverted ? -1.0 : 1.0)),
            () -> (mJoystick.getRawAxis(Constants.Drive.kTurnAxis) * (Constants.Drive.kTurnInverted ? -1.0 : 1.0))));
        */mTransport.setDefaultCommand(new AutoTransport(mTransport, new JoystickButton(mButtonBoard, 16)::get));
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        //new JoystickButton(mJoystick, 3).whenHeld(new Intake(mIntake,  Constants.Intake.kIntakePower));
        //new JoystickButton(mJoystick, 1).whileHeld(new TransportPower(mTransport, 1.0));
        //new JoystickButton(mJoystick, 2).whenHeld(new SpinRoller(mShooter, 2000));
        //new POVButton(mJoystick, 0).whileHeld(new DrivePower(mDrive, Constants.Drive.kCreepSpeed, Constants.Drive.kCreepSpeed));
        //new POVButton(mJoystick, 90).whileHeld(new DrivePower(mDrive, Constants.Drive.kCreepSpeed, -Constants.Drive.kCreepSpeed));
        //new POVButton(mJoystick, 180).whileHeld(new DrivePower(mDrive, -Constants.Drive.kCreepSpeed, -Constants.Drive.kCreepSpeed));
        //new POVButton(mJoystick, 270).whileHeld(new DrivePower(mDrive, -Constants.Drive.kCreepSpeed, Constants.Drive.kCreepSpeed));
        new JoystickButton(mButtonBoard, 2).whileHeld(new Intake(mIntake, Constants.Intake.kIntakePower));
        new JoystickButton(mButtonBoard, 1).whileHeld(new Intake(mIntake, -Constants.Intake.kIntakePower));
        new JoystickButton(mButtonBoard, 15).whenHeld(new SpinRoller(mShooter, 2000));
        new JoystickButton(mButtonBoard, 4).whileHeld(new TransportPower(mTransport, 1.0));
        new JoystickButton(mButtonBoard, 12).whenHeld(new Climb(mClimb, ClimbState.EXTENDED));
        new JoystickButton(mButtonBoard, 13).whenHeld(new Climb(mClimb, ClimbState.VENTED));
        new JoystickButton(mButtonBoard, 14).whenHeld(new Climb(mClimb, ClimbState.RETRACTED));
        new JoystickButton(mButtonBoard, 3).whileHeld(new TransportPower(mTransport, -1.0));
        new JoystickButton(mButtonBoard, 6).whileHeld(new Aim(mDrive, mVision));

    }
    

    public void initDashboard() {
        WPI_TalonFX[] allTalons = mDrive.getAllTalons();

     /*   for (WPI_TalonFX talon : allTalons) {
            talonEntries.add(mSensorLoggerTab.add("Talon " + (talon.getDeviceID()),
                    talon.getMotorOutputVoltage())
                    .withWidget(BuiltInWidgets.kGraph)
                    .getEntry());
        }

        mGyroEntry = mSensorLoggerTab.add("Gyro", mDrive.getGyro()
                .getFusedHeading())
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();

        mOdometer = mSensorLoggerTab.add("Odometer", mDrive.getAverageDistance())
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();
    */}

    public void updateLogger() {
//        Logger.log(Timer.getFPGATimestamp(), "Time");
    }

    public void updateDashboard() {

  /*      if ((Timer.getFPGATimestamp() % Constants.kUpdateRate) / 0.02 < 1) {
            
            Logger.newNoise();
            Logger.logHistory(Logger.addNoise(mDrive.getGyro().getFusedHeading()), "Gyro");
            Logger.logHistory(Logger.addNoise(mDrive.getAverageDistance()), "Odometer");

            for (int i = 0; i < mDrive.getAllTalons().length; i++) {
                Logger.logHistory(Logger.addNoise(mDrive.getAllTalons()[i].getMotorOutputVoltage()), "Talon " + mDrive.getAllTalons()[i].getDeviceID()); 
            }
        }
        */
    }


    public void initCam() {
        UsbCamera backCamera = CameraServer.getInstance().startAutomaticCapture(0);
        MjpegServer backCameraStream = CameraServer.getInstance().startAutomaticCapture(backCamera);
        backCameraStream.setCompression(Constants.Camera.kCameraCompression);
        backCamera.setResolution(Constants.Camera.kCameraResolution.getX(), Constants.Camera.kCameraResolution.getY());
        backCameraStream.setResolution(Constants.Camera.kCameraResolution.getX(), Constants.Camera.kCameraResolution.getY());
        backCamera.setFPS(Constants.Camera.kCameraFPS);
        backCameraStream.setFPS(Constants.Camera.kCameraFPS);
        
    }

    public void startDashboardCapture() {
        if(DriverStation.getInstance().isFMSAttached()) {
            Shuffleboard.startRecording();
        }
    }

    public void stopDashboardCapture() {
        Shuffleboard.startRecording();
    }

    public void changeMode() {
        mVision.setup();
        mDrive.reset();
    }

    public void stopTalons() {
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
        FollowTrajectory.config(Constants.Drive.kDriveFeedforward.ks, Constants.Drive.kDriveFeedforward.kv, Constants.Drive.kDriveFeedforward.ka, Constants.Drive.kB, Constants.Drive.kZeta, Constants.Drive.kTrackWidth, Constants.Drive.kPathPID);

        try {
            Trajectory line = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Line.wpilib.json"));
            Trajectory intake1 = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Intake1.wpilib.json"));
            Trajectory intake2 = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Intake2.wpilib.json"));
            Trajectory turn = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/90Turn.wpilib.json"));
            Trajectory[] paths = { intake1, intake2 };
            return new MoveForward(mDrive, turn);
        } catch (IOException e) {
            System.out.println(e);
            return new InstantCommand();
        }
    
    }

    public Command aim() {
        return new Aim(mDrive, mVision);
    }

    public void printOdo() {
        System.out.println(mDrive.getPose());
    }

    public void printPos() {
        System.out.println(new WheelPositions(mDrive.getPositionLeft(), mDrive.getPositionRight()));
    }

    public void printVel() {
        System.out.println(mDrive.getWheelSpeeds());
    }

    public void updateRPM() {
        mRPM.setDouble(mShooter.getRPM());
    }

    public void printTOF() {
        System.out.println(mTransport.getRange() + ", " + mTransport.getDeviation());
    }
}
