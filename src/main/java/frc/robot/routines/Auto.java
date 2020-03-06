package frc.robot.routines;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class Auto extends SequentialCommandGroup {

    public Auto(DriveSubsystem driveSubsystem, Trajectory path) {
        addCommands(FollowTrajectory.getCommand(driveSubsystem, path, path.getInitialPose()));
    }
}