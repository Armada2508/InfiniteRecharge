package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.MotorConfig;
import frc.robot.Constants;

public class TransportSubsystem extends SubsystemBase {

    private WPI_TalonSRX m_diagTalon;
    private WPI_TalonSRX m_elevTalon;

    public TransportSubsystem(WPI_TalonSRX diagTalon, WPI_TalonSRX elevTalon) {
        m_diagTalon = diagTalon;
        m_elevTalon = elevTalon;
        m_diagTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        m_elevTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MotorConfig.configTalon(m_diagTalon, Constants.kTransportConfig, Constants.kTransportSlot);
        MotorConfig.configTalon(m_elevTalon, Constants.kTransportConfig, Constants.kTransportSlot);
    }

    @Override
    public void periodic() {
        
    }

    public void setDiagPower(double power) {
        m_diagTalon.set(ControlMode.PercentOutput, power);
    }

    public void setElevPower(double power) {
        m_elevTalon.set(ControlMode.PercentOutput, power);
    }
    
    public void setDiagPosition(double position) {
        m_diagTalon.set(ControlMode.Position, position);
    }
    
    public void setElevPosition(double position) {
        m_diagTalon.set(ControlMode.Position, position);
    }
}