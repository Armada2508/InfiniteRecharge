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
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final TransportSubsystem m_transportSubsystem = new TransportSubsystem(new WPI_TalonSRX(0), new WPI_TalonSRX(0));
    private ArrayList<NetworkTableEntry> talonEntries = new ArrayList<NetworkTableEntry>();
    private Joystick m_joystick = new Joystick(Constants.kJoystickPort);
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
        initDashboard();
        m_driveSubsystem.configTalons();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_joystick, 0).whenHeld(new TransportPower(0.5, false, m_transportSubsystem));
        new JoystickButton(m_joystick, 1).whenHeld(new TransportPower(0.5, true, m_transportSubsystem));
        new JoystickButton(m_joystick, 2).whenHeld(new TransportPower(-0.5, false, m_transportSubsystem));
        new JoystickButton(m_joystick, 3).whenHeld(new TransportPower(-0.5, true, m_transportSubsystem));
    }

    public void initDashboard() {
        WPI_TalonFX[] allTalons = m_driveSubsystem.getAllTalons();

        for (WPI_TalonFX talon : allTalons) {
            talonEntries.add(m_sensorLoggerTab.add("Talon " + (talon.getDeviceID()),
                    talon.getMotorOutputVoltage())
                    .withWidget(BuiltInWidgets.kGraph)
                    .getEntry());
        }

        m_gyroEntry = m_sensorLoggerTab.add("Gyro", m_driveSubsystem.getGyro()
                .getFusedHeading())
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();

        m_odometer = m_sensorLoggerTab.add("Odometer", m_driveSubsystem.getAverageDistance())
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();
    }

    public void updateDashboard() {

        if ((Timer.getFPGATimestamp() % Constants.kUpdateRate) / 0.02 < 1) {

            Random noise = new Random();
            m_gyroEntry.setDouble(m_driveSubsystem.getGyro().getFusedHeading() + (noise.nextDouble() / 10000));
            m_odometer.setDouble(m_driveSubsystem.getAverageDistance() + (noise.nextDouble() / 10000));

            int count = 0;
            for (NetworkTableEntry t : talonEntries) {
                t.setDouble(m_driveSubsystem.getAllTalons()[count].getMotorOutputVoltage() + (noise.nextDouble() / 10000));
                count++;
            }
        }
    }

    public void drive() {
        Command driveCommand = new Drive(m_driveSubsystem,
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
        m_driveSubsystem.reset();
    }

    public void stopTalons() {
        m_driveSubsystem.setVoltage(0.0, 0.0);
    }

    public Command getAutonomousCommand() {

        FollowTrajectory followTrajectory = new FollowTrajectory();

        return followTrajectory.getCommand(m_driveSubsystem,
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
        System.out.println(m_driveSubsystem.getPose());
    }

    public void printPos() {
        System.out.println(new WheelPositions(m_driveSubsystem.getPositionLeft(), m_driveSubsystem.getPositionRight()));
    }

    public void printVel() {
        System.out.println(m_driveSubsystem.getWheelSpeeds());
    }
}
