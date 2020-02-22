package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends CommandBase {
    
    IntakeSubsystem m_intakeSubsystem;
    private double m_power;

    public Intake(IntakeSubsystem intakeSubsystem, double power, boolean reversed) {
        m_intakeSubsystem = intakeSubsystem;
        m_power = power * (reversed ? -1.0 : 1.0);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.set(m_power);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.set(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}