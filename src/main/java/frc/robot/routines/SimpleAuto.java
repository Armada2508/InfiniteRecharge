package frc.robot.routines;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.motion.FollowTrajectory;
import frc.robot.commands.DrivePower;
import frc.robot.subsystems.DriveSubsystem;

public class SimpleAuto extends SequentialCommandGroup {

    public SimpleAuto(DriveSubsystem driveSubsystem) {
        addCommands(new DrivePower(driveSubsystem, -0.3, -0.3));
        addCommands(new WaitCommand(2));
        addCommands(new DrivePower(driveSubsystem, 0.0, 0.0));
    }
}