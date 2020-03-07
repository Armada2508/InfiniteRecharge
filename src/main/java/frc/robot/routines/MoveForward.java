package frc.robot.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.motion.FollowTrajectory;
import frc.robot.commands.Aim;
import frc.robot.commands.DrivePower;
import frc.robot.commands.Intake;
import frc.robot.commands.SpinRoller;
import frc.robot.commands.TransportPower;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MoveForward extends SequentialCommandGroup {

    public MoveForward(DriveSubsystem driveSubsystem, Trajectory path) {
        addCommands(
            new InstantCommand(() -> { System.out.println(path.getInitialPose() + ", " + driveSubsystem.getPose()); } ),
            FollowTrajectory.getCommandReversed(driveSubsystem, path, path.getInitialPose())
        );
    }
}