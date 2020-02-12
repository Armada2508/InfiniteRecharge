package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArmSubsystem extends SubsystemBase {
    public Solenoid m_intakeSolenoid;

    public IntakeArmSubsystem(int solenoidChannel) {
        m_intakeSolenoid = new Solenoid(solenoidChannel);
    }

    @Override
    public void periodic() {
        
    }

    public void engage(boolean engaged) {
        m_intakeSolenoid.set(engaged);
    }
}