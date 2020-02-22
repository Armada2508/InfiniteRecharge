package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends CommandBase {
    private ClimbSubsystem m_climbSubsystem;
    private int m_state;


    /**
     * Creates a new climb system
     * @param climbSubsystem The climbSubsystem to use
     * @param state The state of the system(0=retract, 1=vent, 2=extend)
     */
    public Climb(ClimbSubsystem climbSubsystem, int state) {
        m_climbSubsystem = climbSubsystem;
        m_state = state;

        addRequirements(m_climbSubsystem);
    }

    @Override
    public void initialize() {
        switch (m_state) {
            case 0:
                m_climbSubsystem.retract();
                break;
            case 1:
                m_climbSubsystem.vent();
                break;
            case 2:
                m_climbSubsystem.extend();
                break;
        }
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