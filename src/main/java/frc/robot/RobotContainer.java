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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.logger.Logger;
import frc.lib.motion.*;
import frc.robot.commands.*;
import frc.robot.routines.Auto;
import frc.robot.routines.MoveForward;
import frc.robot.routines.SimpleAuto;
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
    private final IntakeSubsystem mFrontIntake = new IntakeSubsystem(Constants.Intake.kFrontIntakeTalon, Constants.Intake.kFrontIntakeInverted);
    private final IntakeSubsystem mBackIntake = new IntakeSubsystem(Constants.Intake.kBackIntakeTalon, Constants.Intake.kBackIntakeInverted);
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
        mDrive.setDefaultCommand(new DriveClosedLoop(mDrive, () -> (mJoystick.getRawAxis(Constants.Drive.kThrottleAxis) * (Constants.Drive.kThrottleInverted ? -1.0 : 1.0)),
            () -> (mJoystick.getRawAxis(Constants.Drive.kTrimAxis) * (Constants.Drive.kTrimInverted ? -1.0 : 1.0)),
            () -> (mJoystick.getRawAxis(Constants.Drive.kTurnAxis) * (Constants.Drive.kTurnInverted ? -1.0 : 1.0))));

    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //new JoystickButton(mjoystick, 1).whenPressed(new Climb(mclimb, true));
        //new JoystickButton(mjoystick, 1).whenReleased(new Climb(mclimb, false));
   /*     new JoystickButton(mJoystick, 11).whenHeld(new Intake(mBackIntake, 0.5, false));
        new JoystickButton(mJoystick, 11).whenHeld(new Intake(mFrontIntake, 0.5, false));
        new JoystickButton(mJoystick, 11).whenHeld(new TransportPower(0.7, true, mTransport));
        new JoystickButton(mJoystick, 11).whenHeld(new TransportPower(0.7, false, mTransport));
        new JoystickButton(mJoystick, 1).whenHeld(new SpinRoller(mShooter, 5750));
        new JoystickButton(mJoystick, 7).whenHeld(new TransportPower(0.5, false, mTransport));
        new JoystickButton(mJoystick, 9).whenHeld(new TransportPower(0.5, true, mTransport));
        new JoystickButton(mJoystick, 8).whenHeld(new TransportPower(-0.375, false, mTransport));
        new JoystickButton(mJoystick, 10).whenHeld(new TransportPower(-0.375, true, mTransport));
        new JoystickButton(mJoystick, 3).whenHeld(new Intake(mFrontIntake, 0.5, false));
        new JoystickButton(mJoystick, 4).whenHeld(new Intake(mBackIntake, 0.5, false));
        new JoystickButton(mJoystick, 5).whenHeld(new Intake(mFrontIntake, 0.5, true));
        new JoystickButton(mJoystick, 6).whenHeld(new Intake(mBackIntake, 0.5, true));
        new JoystickButton(mJoystick, 2).whileHeld(aim());
        new POVButton(mJoystick, 180).whenPressed(new Climb(mClimb, 0));
        new POVButton(mJoystick, 0).whenPressed(new Climb(mClimb, 1));
        new JoystickButton(mJoystick, 12).whenPressed(new Climb(mClimb, 2));
        new POVButton(mJoystick, 90).whenPressed(new SpinColorWheel(mWOF, 3.75));
        new POVButton(mJoystick, 270).whenPressed(new SpinColorWheel(mWOF, -0.125));
*/

        new JoystickButton(mJoystick, 11).whenHeld(new Intake(mFrontIntake, -Constants.Intake.kIntakePower));
        new JoystickButton(mJoystick, 11).whenHeld(new Intake(mBackIntake, -Constants.Intake.kIntakePower));
        new JoystickButton(mJoystick, 11).whenHeld(new TransportPower(mTransport, -Constants.Intake.kIntakePower, true, true));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kSpinUp).whileHeld(new SpinRoller(mShooter, 6400));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kShootSequence).whileHeld(new Aim(mDrive, mVision));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kFeedShooter).whileHeld(new TransportPower(mTransport, 0.8, true, true));
       // new JoystickButton(mButtonBoard, Constants.ButtonBoard.kShootSequence).whenPressed();
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kFrontIntake).whileHeld(new Intake(mFrontIntake, Constants.Intake.kIntakePower));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kBackIntake).whileHeld(new Intake(mBackIntake, Constants.Intake.kIntakePower));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kFrontOutput).whileHeld(new Intake(mFrontIntake, -Constants.Intake.kIntakePower));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kBackOutput).whileHeld(new Intake(mBackIntake, -Constants.Intake.kIntakePower));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kSpinWOF).whenPressed(new SpinColorWheel(mWOF, 3.75));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kWOFLeft).whenPressed(new SpinColorWheel(mWOF, -0.125));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kWOFRight).whenPressed(new SpinColorWheel(mWOF, 0.125));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kWOFLeftSmall).whenPressed(new SpinColorWheel(mWOF, -0.025));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kWOFRightSmall).whenPressed(new SpinColorWheel(mWOF, 0.025));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kClimbExtend).whenPressed(new Climb(mClimb, 2));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kClimbVent).whenPressed(new Climb(mClimb, 1));
        new JoystickButton(mButtonBoard, Constants.ButtonBoard.kClimbRetract).whenPressed(new Climb(mClimb, 0));
        
    
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
        return new SimpleAuto(mDrive, mShooter, mTransport, mFrontIntake, mBackIntake, mVision);
        //return new Aim(mDrive, mVision);
/*
        FollowTrajectory.config(Constants.Drive.kDriveFeedforward.ks, Constants.Drive.kDriveFeedforward.kv, Constants.Drive.kDriveFeedforward.ka, Constants.Drive.kB, Constants.Drive.kZeta, Constants.Drive.kTrackWidth, Constants.Drive.kPathPID);
        try {
            Trajectory line = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Line.wpilib.json"));
            Trajectory intake1 = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Intake1.wpilib.json"));
            Trajectory intake2 = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Intake2.wpilib.json"));
            Trajectory[] paths = { intake1, intake2 };
            //return new SimpleAuto(mDrive);
            //return new Auto(mDrive, mTransport, mShooter, mFrontIntake, mBackIntake, mVision, paths);
            //return new MoveForward(mDrive, line);
        } catch (IOException e) {
            System.out.println(e);
            return new InstantCommand();
        }


        /*
        return new FollowTarget(mdriveSubsystem,
            Constants.kTurn,
            Constants.kThrottle,
            Constants.kMaxFollowOutput,
            Constants.kTargetWidth,
            Constants.kTargetDistance,
            Constants.kLimelightFOV,
            Constants.kLimelighResolution);
*/

    
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
}
