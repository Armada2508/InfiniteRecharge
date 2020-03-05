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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.logger.Logger;
import frc.lib.motion.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
    private final VisionSubsystem mVisionSubsystem = new VisionSubsystem();
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
        mVisionSubsystem.setup();
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
        new JoystickButton(mJoystick, 11).whenHeld(new Intake(mBackIntake, 0.5, false));
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
        new POVButton(mJoystick, 90).whenPressed(new SpinColorWheel(mWOF, 0.125));
        new POVButton(mJoystick, 270).whenPressed(new SpinColorWheel(mWOF, -0.125));

        /*new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kSpinUp).whenPressed(new SpinRoller(mshooter, 4500, Constants.kMaxShooterSlewRate));
      //  new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kAim).whenPressed();
       // new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kFeedShooter).whenPressed();
       // new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kShootSequence).whenPressed();
        new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kFrontIntake).whenPressed(new Intake(mfrontIntake, Constants.kIntakePower, false));
        new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kBackIntake).whenPressed(new Intake(mbackIntake, Constants.kIntakePower, false));
        new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kBothIntake).whenPressed(new ParallelCommandGroup(new Intake(mfrontIntake, Constants.kIntakePower, false), new Intake(mbackIntake, Constants.kIntakePower, false)));
        new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kFrontOutput).whenPressed(new Intake(mfrontIntake, Constants.kIntakePower, true));
        new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kBackOutput).whenPressed(new Intake(mbackIntake, Constants.kIntakePower, true));
        //new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kSpinWOF).whenPressed();
        //new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kIncrementWOF).whenPressed();
        new JoystickButton(mbuttonBoard, Constants.ButtonBoard.kStop).whenPressed(new InstantCommand(() -> { CommandScheduler.getInstance().cancelAll(); }));
     */
    
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

    public void drive() {
        /*        () -> (mJoystick.getRawAxis(Constants.Drive.kThrottleAxis) * (Constants.Drive.kThrottleInverted ? -1.0 : 1.0)),
                () -> (mJoystick.getRawAxis(Constants.Drive.kTrimAxis) * (Constants.Drive.kTrimInverted ? -1.0 : 1.0)),
                () -> (mJoystick.getRawAxis(Constants.Drive.kTurnAxis) * (Constants.Drive.kTurnInverted ? -1.0 : 1.0)));

        driveCommand.schedule();
    */}

    public void initCam() {
        UsbCamera backCamera = CameraServer.getInstance().startAutomaticCapture(0);
        MjpegServer backCameraStream = CameraServer.getInstance().startAutomaticCapture(backCamera);
        backCameraStream.setCompression(Constants.Camera.kCameraCompression);
        backCamera.setResolution(Constants.Camera.kCameraResolution.getX(), Constants.Camera.kCameraResolution.getY());
        backCamera.setFPS(Constants.Camera.kCameraFPS);        
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
        mVisionSubsystem.setup();
        mDrive.reset();
    }

    public void stopTalons() {
        mDrive.setVoltage(0.0, 0.0);
    }

    public Command getAutonomousCommand() {

        return FollowTrajectory.getCommand(mDrive,
            new Pose2d(),
            new Pose2d(5, 0, new Rotation2d()),
            Constants.Drive.kMaxVelocity,
            Constants.Drive.kMaxAcceleration);

        /*
        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Line.wpilib.json"));
            return followTrajectory.getCommand(mdriveSubsystem, trajectory, trajectory.getInitialPose());
        } catch (IOException e) {
            System.out.println(e);
            return new InstantCommand();
        }

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
        return new Aim(mDrive, mVisionSubsystem);
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
