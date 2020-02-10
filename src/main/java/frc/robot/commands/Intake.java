package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends CommandBase {

    private Consumer<Double> m_intake;
    private double m_power;

    public Intake(Consumer<Double> intake, double power, boolean reversed) {
        m_intake = intake;
        m_power = power * (reversed ? -1.0 : 1.0);
    }

    @Override
    public void initialize() {
        m_intake.accept(m_power);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.accept(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}