package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;

public class ShooterSubsystem {

    private WPI_TalonFX m_leftMotor;
    private WPI_TalonFX m_rightMotor;

    public ShooterSubsystem(int solenoidChannel) {
        m_intakeSolenoid = new Solenoid(solenoidChannel);
    }

    public void engage(boolean engaged) {
        m_intakeSolenoid.set(engaged);
    }
}