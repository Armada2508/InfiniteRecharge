package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private Solenoid m_lSolenoid;
    private Solenoid m_rSolenoid;

    public ClimbSubsystem(int lSolenoid, int rSolenoid) {
        m_lSolenoid = new Solenoid(lSolenoid);
        m_rSolenoid = new Solenoid(rSolenoid);
    }

    @Override
    public void periodic() {
        
    }

    public void extend() {
        m_rSolenoid.set(true);
        m_lSolenoid.set(true);
    }

    public void retract() {
        m_rSolenoid.set(false);
        m_lSolenoid.set(false);
    }
}