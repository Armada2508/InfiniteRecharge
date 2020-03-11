package frc.robot.routines;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class MoveForward extends SequentialCommandGroup {

    public MoveForward(DriveSubsystem driveSubsystem, Trajectory path) {
        addCommands(
            new InstantCommand(() -> { System.out.println(path.getInitialPose() + ", " + driveSubsystem.getPose()); } ),
            FollowTrajectory.getCommandReversed(driveSubsystem, path, path.getInitialPose())
        );
    }
}