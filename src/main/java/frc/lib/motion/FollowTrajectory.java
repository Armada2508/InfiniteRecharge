package frc.lib.motion;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;


/**
 * Keeps the robot on a fixed trajectory
 */
public class FollowTrajectory {

    private static SimpleMotorFeedforward mFeedforward;
    private static DifferentialDriveKinematics mKinematics;
    private static RamseteController mController;
    private static PIDController mPidController;

    
    /**
     * @param kS The kS constant
     * @param kV The kV constant
     * @param kA The kA constant
     * @param kP The kP constant
     * @param kI The kI constant
     * @param kD The kD constant
     * @param b The B constant
     * @param zeta The Zeta constant
     * @param trackWidth The width between the tracks of the robot
     */
    public static void config(double kS, double kV, double kA, double b, double zeta, double trackWidth, PIDController pidController) {
        mFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
        mKinematics = new DifferentialDriveKinematics(trackWidth);
        mController = new RamseteController(b, zeta);
        mPidController = pidController;
    }

    /**
     * Returns a RamseteCommand that follows the specified trajectory
     * @param driveSubsystem The DriveSubsystem to use
     * @param trajectory The Trajectory to follow
     * @param zeroPose The position to start relative to
     * @return Returns a RamseteCommand that will follow the specified trajectory with the specified driveSubsystem
     */
    public static Command getCommand(DriveSubsystem driveSubsystem, Trajectory trajectory, Pose2d zeroPose) {
        trajectory = trajectory.relativeTo(zeroPose);
        return new RamseteCommand(
                trajectory,
                driveSubsystem::getPose,          // Equivalent Statement: () -> mdriveSubsystem.getPose(),
                mController,
                mFeedforward,
                mKinematics,
                driveSubsystem::getWheelSpeeds,   // Equivalent Statement: () -> mdriveSubsystem.getWheelSpeeds(),
                mPidController,
                mPidController,
                driveSubsystem::setVoltage,       // Equivalent Statement: (voltsR, voltsL) -> mdriveSubsystem.setVoltage(voltsR, voltsL)
                driveSubsystem);
    }

    /**
     * Returns a RamseteCommand that follows the specified trajectory backwards
     * @param driveSubsystem The DriveSubsystem to use
     * @param trajectory The Trajectory to follow
     * @param zeroPose The position to start relative to
     * @return Returns a RamseteCommand that will follow the specified trajectory with the specified driveSubsystem
     */
    public static Command getCommandReversed(DriveSubsystem driveSubsystem, Trajectory trajectory, Pose2d zeroPose) {
        trajectory = trajectory.relativeTo(zeroPose);
        return new RamseteCommand(
                trajectory,
                driveSubsystem::getPoseReversed,          // Equivalent Statement: () -> mdriveSubsystem.getPose(),
                mController,
                mFeedforward,
                mKinematics,
                driveSubsystem::getWheelSpeedsReverse,   // Equivalent Statement: () -> mdriveSubsystem.getWheelSpeeds(),
                mPidController,
                mPidController,
                driveSubsystem::setVoltageReverse,       // Equivalent Statement: (voltsR, voltsL) -> mdriveSubsystem.setVoltage(voltsR, voltsL)
                driveSubsystem);
    }


    /**
     * Returns a RamseteCommand that follows a generated trajectory
     * @param driveSubsystem The DriveSubsystem to use
     * @param start The position to start at
     * @param end The position to end at
     * @param maxVelocity The maximum velocity of the robot
     * @param maxAcceleration The maximum acceleration of the robot
     * @return Returns a RamseteCommand that will follow the specified trajectory with the specified driveSubsystem
     */
    public static Command getCommand(DriveSubsystem driveSubsystem, Pose2d start, Pose2d end, double maxVelocity, double maxAcceleration, boolean reversed) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setReversed(reversed);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, new ArrayList<Translation2d>(), end,config);
        trajectory = trajectory.relativeTo(trajectory.getInitialPose());
        return getCommand(driveSubsystem, trajectory, trajectory.getInitialPose());
    }
} 