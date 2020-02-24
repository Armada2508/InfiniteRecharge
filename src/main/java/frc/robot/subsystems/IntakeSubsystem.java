package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public WPI_TalonSRX m_intakeTalon;

    public IntakeSubsystem(int talonID, boolean inverted) {
        m_intakeTalon = new WPI_TalonSRX(talonID);
        m_intakeTalon.setInverted(inverted);
    }

    @Override
    public void periodic() {
        
    }

    public void set(double power) {
        m_intakeTalon.set(ControlMode.PercentOutput, power);
    }
}