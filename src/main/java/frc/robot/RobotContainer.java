/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.motion.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;

import java.util.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final TransportSubsystem m_transport = new TransportSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final IntakeSubsystem m_frontIntake = new IntakeSubsystem(Constants.kFrontIntakeTalon, Constants.kFrontIntakeInverted);
    private final IntakeSubsystem m_backIntake = new IntakeSubsystem(Constants.kBackIntakeTalon, Constants.kBackIntakeInverted);
    private final ClimbSubsystem m_climb = new ClimbSubsystem();
    private final ColorWheelSubsystem m_wof = new ColorWheelSubsystem();
    private ArrayList<NetworkTableEntry> talonEntries = new ArrayList<NetworkTableEntry>();
    private Joystick m_joystick = new Joystick(Constants.kJoystickPort);
    private Joystick m_buttonBoard = new Joystick(Constants.ButtonBoard.port);
    private ShuffleboardTab m_sensorLoggerTab = Shuffleboard.getTab("Logger");
    private NetworkTableEntry m_gyroEntry;
    private NetworkTableEntry m_odometer;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        new PneumaticsSubsystem();
    }

    public void robotInit() {
        Logger.configureLoggingAndConfig(this, false);
        initDashboard();
        m_shooter.leftInverted(Constants.kShooterLeftInveted);
        m_shooter.rightInverted(Constants.kShooterRightInverted);
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //new JoystickButton(m_joystick, 1).whenPressed(new Climb(m_climb, true));
        //new JoystickButton(m_joystick, 1).whenReleased(new Climb(m_climb, false));
        //new JoystickButton(m_joystick, 1).whenHeld(new TransportPower(1.0, true, m_transport));
        new JoystickButton(m_joystick, 1).whenHeld(new SpinRoller(m_shooter, 6500, Constants.kMaxShooterSlewRate));
        new JoystickButton(m_joystick, 7).whenHeld(new TransportPower(1.0, false, m_transport));
        new JoystickButton(m_joystick, 9).whenHeld(new TransportPower(1.0, true, m_transport));
        new JoystickButton(m_joystick, 8).whenHeld(new TransportPower(-0.25, false, m_transport));
        new JoystickButton(m_joystick, 10).whenHeld(new TransportPower(-0.25, true, m_transport));
        new JoystickButton(m_joystick, 3).whenHeld(new Intake(m_frontIntake, 0.5, false));
        new JoystickButton(m_joystick, 4).whenHeld(new Intake(m_backIntake, 0.5, false));
        new JoystickButton(m_joystick, 5).whenHeld(new Intake(m_frontIntake, 0.5, true));
        new JoystickButton(m_joystick, 6).whenHeld(new Intake(m_backIntake, 0.5, true));
        new POVButton(m_joystick, 180).whileHeld(new Climb(m_climb, 0));
        new POVButton(m_joystick, -1).whileHeld(new Climb(m_climb, 1));
        new POVButton(m_joystick, 0).whileHeld(new Climb(m_climb, 2));

        /*new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kSpinUp).whenPressed(new SpinRoller(m_shooter, 4500, Constants.kMaxShooterSlewRate));
      //  new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kAim).whenPressed();
       // new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kFeedShooter).whenPressed();
       // new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kShootSequence).whenPressed();
        new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kFrontIntake).whenPressed(new Intake(m_frontIntake, Constants.kIntakePower, false));
        new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kBackIntake).whenPressed(new Intake(m_backIntake, Constants.kIntakePower, false));
        new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kBothIntake).whenPressed(new ParallelCommandGroup(new Intake(m_frontIntake, Constants.kIntakePower, false), new Intake(m_backIntake, Constants.kIntakePower, false)));
        new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kFrontOutput).whenPressed(new Intake(m_frontIntake, Constants.kIntakePower, true));
        new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kBackOutput).whenPressed(new Intake(m_backIntake, Constants.kIntakePower, true));
        //new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kSpinWOF).whenPressed();
        //new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kIncrementWOF).whenPressed();
        new JoystickButton(m_buttonBoard, Constants.ButtonBoard.kStop).whenPressed(new InstantCommand(() -> { CommandScheduler.getInstance().cancelAll(); }));
     */
    
    }
    

    public void initDashboard() {
        WPI_TalonFX[] allTalons = m_drive.getAllTalons();

        for (WPI_TalonFX talon : allTalons) {
            talonEntries.add(m_sensorLoggerTab.add("Talon " + (talon.getDeviceID()),
                    talon.getMotorOutputVoltage())
                    .withWidget(BuiltInWidgets.kGraph)
                    .getEntry());
        }

        m_gyroEntry = m_sensorLoggerTab.add("Gyro", m_drive.getGyro()
                .getFusedHeading())
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();

        m_odometer = m_sensorLoggerTab.add("Odometer", m_drive.getAverageDistance())
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();
    }

    public void updateLogger() {
        Logger.updateEntries();
    }

    public void updateDashboard() {

        if ((Timer.getFPGATimestamp() % Constants.kUpdateRate) / 0.02 < 1) {

            Random noise = new Random();
            m_gyroEntry.setDouble(m_drive.getGyro().getFusedHeading() + (noise.nextDouble() / 10000));
            m_odometer.setDouble(m_drive.getAverageDistance() + (noise.nextDouble() / 10000));

            int count = 0;
            for (NetworkTableEntry t : talonEntries) {  
                t.setDouble(m_drive.getAllTalons()[count].getMotorOutputVoltage() + (noise.nextDouble() / 10000));
                count++;
            }
        }
    }

    public void drive() {
        Command driveCommand = new DriveClosedLoop(m_drive,
                () -> (m_joystick.getRawAxis(Constants.kThrottleAxis) * (Constants.kThrottleInverted ? -1.0 : 1.0)),
                () -> (m_joystick.getRawAxis(Constants.kTrimAxis) * (Constants.kTrimInverted ? -1.0 : 1.0)),
                () -> (m_joystick.getRawAxis(Constants.kTurnAxis) * (Constants.kTurnInverted ? -1.0 : 1.0)),
                Constants.kMaxPower,
                Constants.kTurnRatio,
                Constants.kTrimRatio);

        driveCommand.schedule();
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
        m_drive.reset();
    }

    public void stopTalons() {
        m_drive.setVoltage(0.0, 0.0);
    }

    public Command getAutonomousCommand() {

        FollowTrajectory followTrajectory = new FollowTrajectory();

        return followTrajectory.getCommand(m_drive,
            new Pose2d(),
            new Pose2d(5, 0, new Rotation2d()),
            Constants.kMaxVelocity,
            Constants.kMaxAcceleration);

        /*
        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(Filesystem.getDeployDirectory().toString(), "/paths/output/Line.wpilib.json"));
            return followTrajectory.getCommand(m_driveSubsystem, trajectory, trajectory.getInitialPose());
        } catch (IOException e) {
            System.out.println(e);
            return new InstantCommand();
        }

        return new FollowTarget(m_driveSubsystem,
            Constants.kTurn,
            Constants.kThrottle,
            Constants.kMaxFollowOutput,
            Constants.kTargetWidth,
            Constants.kTargetDistance,
            Constants.kLimelightFOV,
            Constants.kLimelighResolution);
    */
    }

    public void printOdo() {
        System.out.println(m_drive.getPose());
    }

    public void printPos() {
        System.out.println(new WheelPositions(m_drive.getPositionLeft(), m_drive.getPositionRight()));
    }

    public void printVel() {
        System.out.println(m_drive.getWheelSpeeds());
    }

    public void printRPM() {
        System.out.println(m_shooter.getRPM());
    }
}
