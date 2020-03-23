package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.FeedbackConstants;
import frc.lib.config.MotionMagicConfig;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class TransportSubsystem extends SubsystemBase {

    private WPI_TalonSRX mTransportTalon;
    private TimeOfFlight mIntake;
    private TimeOfFlight mShooter;
    private double mSetpoint;

    public TransportSubsystem() {
        mTransportTalon = new WPI_TalonSRX(Constants.Transport.kTalon);
        mTransportTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MotorConfig.resetTalon(mTransportTalon);
        FeedbackConstants.config(mTransportTalon, Constants.Transport.kVelocityFeedbackConstants, Constants.Transport.kVelocitySlot);
        FeedbackConstants.config(mTransportTalon, Constants.Transport.kFeedbackConstants, Constants.Transport.kSlot);
        MotorConfig.config(mTransportTalon, Constants.Transport.kConfig);
        MotionMagicConfig.config(mTransportTalon, Constants.Transport.kMMConfig);

        mIntake = new TimeOfFlight(Constants.Transport.kIntakeTOF);
        mShooter = new TimeOfFlight(Constants.Transport.kShooterTOF);

        setTOFMode(Constants.Transport.kRangingMode, Constants.Transport.kTOFSampleTime);
    }

    @Override
    public void periodic() {
    }

    public int getRawPosition() {
        return mTransportTalon.getSelectedSensorPosition();
    }

    public double getPosition() {
        return EncoderUtil.toDistance(getRawPosition(), Constants.Transport.kFeedbackConfig.getEpr(), Constants.Transport.kFeedbackConfig.getGearRatio(), Constants.Transport.kPulleyDiameter);
    }

    public int getRawVelocity() {
        return mTransportTalon.getSelectedSensorVelocity();
    }

    public double getVelocity() {
        return EncoderUtil.toVelocity(getRawVelocity(), Constants.Transport.kFeedbackConfig.getEpr(), Constants.Transport.kFeedbackConfig.getGearRatio(), Constants.Transport.kPulleyDiameter, Constants.Transport.kVelocitySampleTime);
    }

    public void setPower(double power) {
        mTransportTalon.set(ControlMode.PercentOutput, power);
    }

    public void setVelocity(double velocity) {
        mTransportTalon.selectProfileSlot(Constants.Transport.kVelocitySlot, 0);
        mTransportTalon.set(ControlMode.Velocity, fromVelocity(velocity));
    }

    public void setRawVelocity(int velocity) {
        mTransportTalon.selectProfileSlot(Constants.Transport.kVelocitySlot, 0);
        mTransportTalon.set(ControlMode.Velocity, velocity);
    }

    public double toVelocity(int velocity) {
        return EncoderUtil.toVelocity(velocity, Constants.Transport.kFeedbackConfig.getEpr(), Constants.Transport.kFeedbackConfig.getGearRatio(), Constants.Transport.kPulleyDiameter, Constants.Transport.kVelocitySampleTime);
    }

    public int fromVelocity(double velocity) {
        return (int)EncoderUtil.fromVelocity(velocity, Constants.Transport.kFeedbackConfig.getEpr(), Constants.Transport.kFeedbackConfig.getGearRatio(), Constants.Transport.kPulleyDiameter, Constants.Transport.kVelocitySampleTime);
    }

    public double toDistance(int distance) {
        return EncoderUtil.toDistance(distance, Constants.Transport.kFeedbackConfig.getEpr(), Constants.Transport.kFeedbackConfig.getGearRatio(), Constants.Transport.kPulleyDiameter);
    }

    public int fromDistance(double distance) {
        return (int)EncoderUtil.fromDistance(distance, Constants.Transport.kFeedbackConfig.getEpr(), Constants.Transport.kFeedbackConfig.getGearRatio(), Constants.Transport.kPulleyDiameter);
    }

    public void setRawPosition(double position) {
        mTransportTalon.selectProfileSlot(Constants.Transport.kSlot, 0);
        mTransportTalon.set(ControlMode.MotionMagic, position);
    }

    public void setPosition(double position) {
        mTransportTalon.selectProfileSlot(Constants.Transport.kSlot, 0);
        mTransportTalon.set(ControlMode.MotionMagic, fromDistance(position));
        mSetpoint = position;
    }
    
    public void zeroPosition() {
        mTransportTalon.setSelectedSensorPosition(0);
    }
    
    public void incrementPosition(double increment) {
        setPosition(getPosition()+increment);
    }

    public double[] sense() {
        double[] distance =  { mIntake.getRange(), mShooter.getRange() };
        return distance;
    }

    public double[] deviation() {
        double[] deviation =  { mIntake.getRangeSigma(), mShooter.getRangeSigma() };
        return deviation;
    }

    public boolean[] isValid() {
        boolean[] isValid =  { mIntake.isRangeValid(), mShooter.isRangeValid() };
        return isValid;
    }

    public boolean[] isBall() {
        double[] distance = sense();
        double[] deviation = deviation();
        boolean[] isBall = new boolean[distance.length];
        for (int i = 0; i < isBall.length; i++) {
            isBall[i] = (distance[i] < Constants.Transport.kMaxDistance) && (deviation[i] < Constants.Transport.kMaxDeviation);
        }
        return isBall;
    }

    public boolean[] isBall(double maxDistance, double maxDeviation) {
        double[] distance = sense();
        double[] deviation = deviation();
        boolean[] isBall = new boolean[distance.length];
        for (int i = 0; i < isBall.length; i++) {
            isBall[i] = (distance[i] < maxDistance) && (deviation[i] < maxDeviation);
        }
        return isBall;
    }

    public void setTOFMode(RangingMode rangingMode, double sampleTime) {
        mIntake.setRangingMode(rangingMode, sampleTime);
        mShooter.setRangingMode(rangingMode, sampleTime);
    }

    public double getRange() {
        return mIntake.getRange();
    }
    
    public double getDeviation() {
        return mIntake.getRangeSigma();
    }

    public boolean getValid() {
        return mIntake.isRangeValid();
    }

    public boolean motionMagicDone() {
        boolean usingMotionMagic = mTransportTalon.getControlMode() == ControlMode.MotionMagic;
        return !usingMotionMagic || (Math.abs(getPosition() - mSetpoint) < Constants.Transport.kThreshold);
    }
}