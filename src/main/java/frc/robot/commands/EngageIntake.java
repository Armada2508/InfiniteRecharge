package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class EngageIntake extends CommandBase {
    
    private Consumer<Boolean> m_engageIntake;
    private boolean m_engaged;

    public EngageIntake(Consumer<Boolean> engageIntake, boolean engaged) {
        m_engageIntake = engageIntake;
        m_engaged = engaged;
    }
    
    @Override
    public void initialize() {
        m_engageIntake.accept(m_engaged);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_engageIntake.accept(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}