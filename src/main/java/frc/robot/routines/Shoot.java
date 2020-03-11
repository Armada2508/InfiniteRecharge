package frc.robot.routines;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.SpinRoller;
import frc.robot.commands.TransportPower;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

public class Shoot extends SequentialCommandGroup {

    public Shoot(ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem, IntakeSubsystem frontIntake, IntakeSubsystem backIntake) {
        addCommands(new ParallelRaceGroup(
            new SpinRoller(shooterSubsystem, 6400),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(2.0),
                    new Intake(frontIntake, 1),
                    new TransportPower(transportSubsystem, 1.0)
                ),
                new ParallelRaceGroup(
                    new WaitCommand(2.0),
                    new Intake(backIntake, 1),
                    new TransportPower(transportSubsystem, 1.0)
                )
            )
        ));
    }
}