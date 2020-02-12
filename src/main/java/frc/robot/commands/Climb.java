package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb extends CommandBase {
    private Runnable m_actuationRunnable;

    public Climb(Runnable actuationRunnable) {
        m_actuationRunnable = actuationRunnable;
    }

    @Override
    public void initialize() {
        m_actuationRunnable.run();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}