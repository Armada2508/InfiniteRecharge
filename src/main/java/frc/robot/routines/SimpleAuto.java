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

    public SimpleAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem, IntakeSubsystem frontIntake, IntakeSubsystem backIntake, VisionSubsystem visionSubsystem) {
        addCommands(new ParallelRaceGroup(
            new SpinRoller(shooterSubsystem, 6400),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new DrivePower(driveSubsystem, -0.3, -0.3),
                new WaitCommand(1.1),
                //new WaitCommand(0.8),
                new DrivePower(driveSubsystem, 0.0, 0.0),
                new WaitCommand(1.75),
                new ParallelRaceGroup(
                    new Aim(driveSubsystem, visionSubsystem),
                    new WaitCommand(1.5)
                ),
                new ParallelRaceGroup(
                    new TransportPower(transportSubsystem, 0.75),
                    new WaitCommand(2.0)
                )
                /*new ParallelRaceGroup(
                    new DrivePower(driveSubsystem, -0.3, -0.3),
                    new WaitCommand(0.8),
                    new Intake(backIntake, 1.0),
                    new WaitCommand(1.5)
                ),
                new DrivePower(driveSubsystem, 0.0, 0.0),
                new ParallelRaceGroup(
                    new Aim(driveSubsystem, visionSubsystem),
                    new WaitCommand(2.0)
                ),
                new TransportPower(transportSubsystem, 0.75, true, true)*/
            )
        ));
        addCommands(new SpinRoller(shooterSubsystem, 0));
        

    }
}