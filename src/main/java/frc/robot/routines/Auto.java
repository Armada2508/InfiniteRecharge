package frc.robot.routines;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.motion.FollowTrajectory;
import frc.robot.commands.Aim;
import frc.robot.commands.Intake;
import frc.robot.commands.SpinRoller;
import frc.robot.commands.TransportPower;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Auto extends SequentialCommandGroup {

    public Auto(DriveSubsystem driveSubsystem, TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem frontIntake, IntakeSubsystem backIntake, VisionSubsystem visionSubsystem, Trajectory[] paths) {
        addCommands(new ParallelCommandGroup(
            // ===================
            //    Start of Auto
            // ==================

            // Spin Up Shooter
            new SpinRoller(shooterSubsystem, 5800),
            new SequentialCommandGroup(
                // Spin Intakes for 1 second
                new ParallelRaceGroup(
                    new Intake(frontIntake, 1),
                    new Intake(backIntake, 1),
                    new WaitCommand(1.0)
                ),
                FollowTrajectory.getCommandReversed(driveSubsystem, paths[0], paths[0].getInitialPose()),
                //new Aim(driveSubsystem, visionSubsystem),
                new Shoot(shooterSubsystem, transportSubsystem, frontIntake, backIntake),
                new ParallelRaceGroup(
                    new SpinRoller(shooterSubsystem, 6400),
                    new Intake(backIntake, 1),
                    new TransportPower(transportSubsystem, 0.5, true, true),
                    FollowTrajectory.getCommandReversed(driveSubsystem, paths[1], paths[1].getInitialPose())
                ),
                // Move Backwards
                FollowTrajectory.getCommand(driveSubsystem, paths[1], paths[1].getInitialPose()),
                //new Aim(driveSubsystem, visionSubsystem),
                new Shoot(shooterSubsystem, transportSubsystem, frontIntake, backIntake)
            )
        ));
    }
}