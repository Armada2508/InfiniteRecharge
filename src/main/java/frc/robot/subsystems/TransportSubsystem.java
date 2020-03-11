package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.MotionMagicConfig;
import frc.lib.config.MotorConfig;
import frc.lib.motion.EncoderUtil;
import frc.robot.Constants;

public class TransportSubsystem extends SubsystemBase {

    private WPI_TalonSRX mTransportTalon;
    private TimeOfFlight mIntake;
    private TimeOfFlight mShooter;

    public TransportSubsystem() {
        mTransportTalon = new WPI_TalonSRX(Constants.Transport.kTransportTalon);
        mTransportTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MotorConfig.configTalon(mTransportTalon, Constants.Transport.kTransportConfig, Constants.Transport.kTransportSlot);
        MotionMagicConfig.configTalon(mTransportTalon, Constants.Transport.kTransportMMConfig);

        mIntake = new TimeOfFlight(Constants.Transport.kIntakeTOF);
        mShooter = new TimeOfFlight(Constants.Transport.kShotoerTOF);

        setTOFMode(Constants.Transport.kRangingMode, Constants.Transport.kTOFSampleTime);
    }

    @Override
    public void periodic() {
    }

    public double getRawPosition() {
        return mTransportTalon.getSelectedSensorPosition();
    }

    public double getPosition() {
        return EncoderUtil.toDistance(mTransportTalon.getSelectedSensorPosition(), Constants.Transport.kTransportEncoderUnitsPerRev, 1.0, Constants.Transport.kPulleyDiameter);
    }

    public void setPower(double power) {
        mTransportTalon.set(ControlMode.PercentOutput, power);
    }

    public void setRawPosition(double position) {
        mTransportTalon.set(ControlMode.MotionMagic, position);
    }

    public void setPosition(double position) {
        mTransportTalon.set(ControlMode.MotionMagic, EncoderUtil.fromDistance(getPosition(), Constants.Transport.kTransportEncoderUnitsPerRev, 1.0, Constants.Transport.kPulleyDiameter));
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
        mIntake.setRangingMode(rangingMode, sampleTime);
        mShooter.setRangingMode(rangingMode, sampleTime);
    }
}