package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class TransportSubsystem extends SubsystemBase {

    private WPI_TalonSRX m_diagTalon;
    private WPI_TalonSRX m_elevTalon;
    private TimeOfFlight m_fIntake;
    private TimeOfFlight m_bIntake;
    private TimeOfFlight m_interface;
    private TimeOfFlight m_shooter;

    public TransportSubsystem() {
        m_diagTalon = new WPI_TalonSRX(Constants.kDiagonalTalon);
        m_elevTalon = new WPI_TalonSRX(Constants.kElevatorTalon);
        m_diagTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        m_elevTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MotorConfig.configTalon(m_diagTalon, Constants.kTransportConfig, Constants.kTransportSlot);
        MotorConfig.configTalon(m_elevTalon, Constants.kTransportConfig, Constants.kTransportSlot);

        m_fIntake = new TimeOfFlight(Constants.kFIntakeTofID);
        m_bIntake = new TimeOfFlight(Constants.kBIntakeTofID);
        m_interface = new TimeOfFlight(Constants.kInterfaceTofID);
        m_shooter = new TimeOfFlight(Constants.kShooterTofID);

        setTOFMode(Constants.kRangingMode, Constants.kTOFSampleTime);
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
        return EncoderUtil.toDistance(m_elevTalon.getSelectedSensorPosition(), Constants.kTransportEncoderUnitsPerRev, 1.0, Constants.kPulleyDiameter);
    }

    public double getDiagPosition() {
        return EncoderUtil.toDistance(m_elevTalon.getSelectedSensorPosition(), Constants.kTransportEncoderUnitsPerRev, 1.0, Constants.kPulleyDiameter);
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
        m_elevTalon.set(ControlMode.Position, EncoderUtil.fromDistance(getElevPosition(), Constants.kTransportEncoderUnitsPerRev, 1.0, Constants.kPulleyDiameter));
    }

    public void setDiagPosition(double position) {
        m_diagTalon.set(ControlMode.Position, EncoderUtil.fromDistance(getDiagPosition(), Constants.kTransportEncoderUnitsPerRev, 1.0, Constants.kPulleyDiameter));
    }
    
    public void zeroElevPosition() {
        m_elevTalon.setSelectedSensorPosition(0);   
    }
    
    public void zeroDiagPosition() {
        m_diagTalon.setSelectedSensorPosition(0);
    }

    public void incrementElevPosition(double increment) {
        setElevPosition(getElevPosition()+increment);
    }
    
    public void incrementDiagPosition(double increment) {
       setDiagPosition(getDiagPosition()+increment);
    }

    public double[] sense() {
        double[] distance =  { m_fIntake.getRange(), m_bIntake.getRange(), m_interface.getRange(), m_shooter.getRange() };
        return distance;
    }

    public double[] deviation() {
        double[] deviation =  { m_fIntake.getRangeSigma(), m_bIntake.getRangeSigma(), m_interface.getRangeSigma(), m_shooter.getRangeSigma() };
        return deviation;
    }

    public boolean[] isValid() {
        boolean[] isValid =  { m_fIntake.isRangeValid(), m_bIntake.isRangeValid(), m_interface.isRangeValid(), m_shooter.isRangeValid() };
        return isValid;
    }

    public boolean[] isBall() {
        double[] distance = sense();
        double[] deviation = deviation();
        boolean[] isValid = isValid();
        boolean[] isBall = new boolean[distance.length];
        for (int i = 0; i < isBall.length; i++) {
            isBall[i] = (distance[i] < Constants.kMaxDistance) && (deviation[i] < Constants.kMaxDeviation) && isValid[i];
        }
        return isBall;
    }

    public boolean[] isBall(double maxDistance, double maxDeviation) {
        double[] distance = sense();
        double[] deviation = deviation();
        boolean[] isValid = isValid();
        boolean[] isBall = new boolean[distance.length];
        for (int i = 0; i < isBall.length; i++) {
            isBall[i] = (distance[i] < maxDistance) && (deviation[i] < maxDeviation) && isValid[i];
        }
        return isBall;
    }

    public void setTOFMode(RangingMode rangingMode, double sampleTime) {
        m_fIntake.setRangingMode(rangingMode, sampleTime);
        m_bIntake.setRangingMode(rangingMode, sampleTime);
        m_interface.setRangingMode(rangingMode, sampleTime);
        m_shooter.setRangingMode(rangingMode, sampleTime);
    }
}