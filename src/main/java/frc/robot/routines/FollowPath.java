package frc.robot.routines;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPath extends SequentialCommandGroup {

    public FollowPath(DriveSubsystem driveSubsystem, Trajectory path) {
        // ===================
        //    Start of Auto
        // ===================
        addCommands(
            // Follow Path
            FollowTrajectory.getCommand(driveSubsystem, path, path.getInitialPose())
        );
    }
}