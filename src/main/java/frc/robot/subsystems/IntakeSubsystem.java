package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public WPI_TalonSRX m_intakeTalon;

    public IntakeSubsystem(int talonID) {
        m_intakeTalon = new WPI_TalonSRX(talonID);
    }

    @Override
    public void periodic() {
        
    }

    public void set(double power) {
        System.out.println(power);
        m_intakeTalon.set(ControlMode.PercentOutput, power);
    }
}