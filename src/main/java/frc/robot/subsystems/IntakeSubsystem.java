package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SlewRateLimiter;

public class IntakeSubsystem {

    public WPI_TalonSRX m_intakeTalon;
    public SlewRateLimiter m_slewRateLimiter;

    public IntakeSubsystem(int talonID) {
        m_intakeTalon = new WPI_TalonSRX(talonID);
    }
    
    public IntakeSubsystem(int talonID, double slewRateLimit) {
        m_intakeTalon = new WPI_TalonSRX(talonID);
        m_slewRateLimiter = new SlewRateLimiter(slewRateLimit);
    }

    public void set(double power) {
        m_intakeTalon.set(ControlMode.PercentOutput, power);
    }
}