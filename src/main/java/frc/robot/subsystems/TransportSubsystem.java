package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransportSubsystem extends SubsystemBase {

    private WPI_TalonSRX[] m_talons;

    public TransportSubsystem(int... talonID) {
        m_talons = new WPI_TalonSRX[talonID.length];
        for (int i = 0; i < talonID.length; i++) {
            m_talons[i] = new WPI_TalonSRX(talonID[i]);
        }
    }

    @Override
    public void periodic() {
        
    }

    public void set(double power, int talon) {
        m_talons[talon].set(ControlMode.PercentOutput, power);
    }

    public void setPosition(double position, int talon) {
        m_talons[talon].set(ControlMode.Position, position);
    }
}