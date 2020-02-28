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

    private WPI_TalonSRX mDiagTalon;
    private WPI_TalonSRX mElevTalon;
    private TimeOfFlight mFIntake;
    private TimeOfFlight mBIntake;
    private TimeOfFlight mInterface;
    private TimeOfFlight mShooter;

    public TransportSubsystem() {
        mDiagTalon = new WPI_TalonSRX(Constants.Transport.kDiagonalTalon);
        mElevTalon = new WPI_TalonSRX(Constants.Transport.kElevatorTalon);
        mDiagTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mElevTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MotorConfig.configTalon(mDiagTalon, Constants.Transport.kTransportConfig, Constants.Transport.kTransportSlot);
        MotorConfig.configTalon(mElevTalon, Constants.Transport.kTransportConfig, Constants.Transport.kTransportSlot);

        mFIntake = new TimeOfFlight(Constants.Transport.kFIntakeTofID);
        mBIntake = new TimeOfFlight(Constants.Transport.kBIntakeTofID);
        mInterface = new TimeOfFlight(Constants.Transport.kInterfaceTofID);
        mShooter = new TimeOfFlight(Constants.Transport.kShooterTofID);

        setTOFMode(Constants.Transport.kRangingMode, Constants.Transport.kTOFSampleTime);
    }

    @Override
    public void periodic() {
    }

    public double getRawElevPosition() {
        return mDiagTalon.getSelectedSensorPosition();
    }

    public double getRawDiagPosition() {
        return mDiagTalon.getSelectedSensorPosition();
    }

    public double getElevPosition() {
        return EncoderUtil.toDistance(mElevTalon.getSelectedSensorPosition(), Constants.Transport.kTransportEncoderUnitsPerRev, 1.0, Constants.Transport.kPulleyDiameter);
    }

    public double getDiagPosition() {
        return EncoderUtil.toDistance(mElevTalon.getSelectedSensorPosition(), Constants.Transport.kTransportEncoderUnitsPerRev, 1.0, Constants.Transport.kPulleyDiameter);
    }

    public void setElevPower(double power) {
        mElevTalon.set(ControlMode.PercentOutput, power);
    }

    public void setDiagPower(double power) {
        mDiagTalon.set(ControlMode.PercentOutput, power);
    }

    public void setRawElevPosition(double position) {
        mElevTalon.set(ControlMode.Position, position);
    }

    public void setRawDiagPosition(double position) {
        mDiagTalon.set(ControlMode.Position, position);
    }

    public void setElevPosition(double position) {
        mElevTalon.set(ControlMode.Position, EncoderUtil.fromDistance(getElevPosition(), Constants.Transport.kTransportEncoderUnitsPerRev, 1.0, Constants.Transport.kPulleyDiameter));
    }

    public void setDiagPosition(double position) {
        mDiagTalon.set(ControlMode.Position, EncoderUtil.fromDistance(getDiagPosition(), Constants.Transport.kTransportEncoderUnitsPerRev, 1.0, Constants.Transport.kPulleyDiameter));
    }
    
    public void zeroElevPosition() {
        mElevTalon.setSelectedSensorPosition(0);   
    }
    
    public void zeroDiagPosition() {
        mDiagTalon.setSelectedSensorPosition(0);
    }

    public void incrementElevPosition(double increment) {
        setElevPosition(getElevPosition()+increment);
    }
    
    public void incrementDiagPosition(double increment) {
       setDiagPosition(getDiagPosition()+increment);
    }

    public double[] sense() {
        double[] distance =  { mFIntake.getRange(), mBIntake.getRange(), mInterface.getRange(), mShooter.getRange() };
        return distance;
    }

    public double[] deviation() {
        double[] deviation =  { mFIntake.getRangeSigma(), mBIntake.getRangeSigma(), mInterface.getRangeSigma(), mShooter.getRangeSigma() };
        return deviation;
    }

    public boolean[] isValid() {
        boolean[] isValid =  { mFIntake.isRangeValid(), mBIntake.isRangeValid(), mInterface.isRangeValid(), mShooter.isRangeValid() };
        return isValid;
    }

    public boolean[] isBall() {
        double[] distance = sense();
        double[] deviation = deviation();
        boolean[] isValid = isValid();
        boolean[] isBall = new boolean[distance.length];
        for (int i = 0; i < isBall.length; i++) {
            isBall[i] = (distance[i] < Constants.Transport.kMaxDistance) && (deviation[i] < Constants.Transport.kMaxDeviation) && isValid[i];
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
        mFIntake.setRangingMode(rangingMode, sampleTime);
        mBIntake.setRangingMode(rangingMode, sampleTime);
        mInterface.setRangingMode(rangingMode, sampleTime);
        mShooter.setRangingMode(rangingMode, sampleTime);
    }
}