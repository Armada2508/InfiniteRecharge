package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends CommandBase {
    
    private IntakeSubsystem mIntakeSubsystem;
    private double mPower;

    public Intake(IntakeSubsystem intakeSubsystem, double power, boolean reversed) {
        mIntakeSubsystem = intakeSubsystem;
        mPower = power * (reversed ? -1.0 : 1.0);
    }

    @Override
    public void initialize() {
        mIntakeSubsystem.set(mPower);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        mIntakeSubsystem.set(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}