package frc.robot.routines;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class CommandGroupTest extends SequentialCommandGroup {

    public CommandGroupTest(DriveSubsystem driveSubsystem, Trajectory[] paths) {
        addCommands(FollowTrajectory.getCommand(driveSubsystem, paths[0], driveSubsystem.getPose()));
    }
}