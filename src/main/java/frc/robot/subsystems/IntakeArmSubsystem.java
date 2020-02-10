package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;

public class IntakeArmSubsystem {
    public Solenoid m_intakeSolenoid;

    public IntakeArmSubsystem(int solenoidChannel) {
        m_intakeSolenoid = new Solenoid(solenoidChannel);
    }

    public void engage(boolean engaged) {
        m_intakeSolenoid.set(engaged);
    }
}