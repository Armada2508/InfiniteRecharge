package frc.robot.routines;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(DriveSubsystem driveSubsystem, Trajectory[] paths) {
        addCommands(FollowTrajectory.getCommand(driveSubsystem, paths[0], paths[0].getInitialPose()));
    }
}