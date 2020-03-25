package frc.robot.routines;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class MoveForward extends SequentialCommandGroup {

    public MoveForward(DriveSubsystem driveSubsystem, Trajectory path) {
        addCommands(
            FollowTrajectory.getCommand(driveSubsystem, path, path.getInitialPose())
        );
    }
}