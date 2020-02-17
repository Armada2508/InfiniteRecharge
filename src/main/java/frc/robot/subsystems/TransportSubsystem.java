package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class TransportSubsystem extends SubsystemBase {

    private WPI_TalonSRX m_diagTalon;
    private WPI_TalonSRX m_elevTalon;
    private Encoder m_diagEncoder;
    private Encoder m_elevEncoder;

    public TransportSubsystem(int diagonalTalonID, int elevatorTalonID, int diagonalEncoderPort, int elevatorEncoderPort) {
        m_diagTalon = new WPI_TalonSRX(Constants.kDiagonalMotorPort);
        m_elevTalon = new WPI_TalonSRX(Constants.kElevatorMotorPort);
        m_diagEncoder = new Encoder(Constants.kDiagonalEncoderPortA, Constants.kDiagonalEncoderPortB);
        m_elevEncoder = new Encoder(Constants.kElevatorEncoderPortA, Constants.kElevatorEncoderPortB);
    }

    @Override
    public void periodic() {
        
    }

    public double getRawElevPosition() {
        return m_diagTalon.getSelectedSensorPosition();
    }

    public double getRawDiagPosition() {
        return m_diagTalon.getSelectedSensorPosition();
    }

    public double getElevPosition() {
        return EncoderUtil.toDistance(m_elevEncoder.get(), Constants.kTransportEncoderUnitsPerRev, 1.0, Constants.kElevatorPulleyDiameter);
    }

    public double getDiagPosition() {
        return EncoderUtil.toDistance(m_diagEncoder.get(), Constants.kTransportEncoderUnitsPerRev, 1.0, Constants.kDiagonalPulleyDiameter);
    }

    public void setElevPower(double power) {
        m_elevTalon.set(ControlMode.PercentOutput, power);
    }

    public void setDiagPower(double power) {
        m_diagTalon.set(ControlMode.PercentOutput, power);
    }

    public void setRawElevPosition(double position) {
        m_elevTalon.set(ControlMode.Position, position);
    }

    public void setRawDiagPosition(double position) {
        m_diagTalon.set(ControlMode.Position, position);
    }

    public void setElevPosition(double position) {
        m_elevTalon.set(ControlMode.Position, EncoderUtil.fromDistance(getElevPosition(), Constants.kTransportEncoderUnitsPerRev, 1.0, Constants.kElevatorPulleyDiameter));
    }

    public void setDiagPosition(double position) {
        m_diagTalon.set(ControlMode.Position, EncoderUtil.fromDistance(getDiagPosition(), Constants.kTransportEncoderUnitsPerRev, 1.0, Constants.kDiagonalPulleyDiameter));
    }

    public void incrementElevPosition(double increment) {
        setElevPosition(getElevPosition()+increment);
    }
    
    public void incrementDiagPosition(double increment) {
       setDiagPosition(getDiagPosition()+increment);
    }
}