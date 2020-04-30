package frc.robot.routines;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Aim;
import frc.robot.commands.DrivePower;
import frc.robot.commands.SpinRoller;
import frc.robot.commands.TransportPower;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SimpleAuto extends SequentialCommandGroup {

    public SimpleAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem, IntakeSubsystem intakeSubsystem, VisionSubsystem visionSubsystem) {
        // ===================
        //    Start of Auto
        // ===================
        addCommands(new ParallelRaceGroup(
            // Spin up shooter
            new SpinRoller(shooterSubsystem, 6400),
            new SequentialCommandGroup(
                // Wait for shooter to spin up
                new WaitCommand(0.5),
                // Drive backwards
                new DrivePower(driveSubsystem, -0.3, -0.3),
                new WaitCommand(1.1),
                // Stop and wait
                new DrivePower(driveSubsystem, 0.0, 0.0),
                new WaitCommand(1.75),
                // Wait for 1.5 seconds
                new ParallelRaceGroup(
                    new Aim(driveSubsystem, visionSubsystem),
                    new WaitCommand(1.5)
                ),
                // Feed balls
                new ParallelRaceGroup(
                    new TransportPower(transportSubsystem, 0.75),
                    new WaitCommand(2.0)
                )
            )
        ));
        // Spin down shooter
        addCommands(new SpinRoller(shooterSubsystem, 0));
        

    }
}