package frc.lib.config;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class MotorConfig {
    private final double mP;
    private final double mI;
    private final double mD;
    private final double mF;
    private final double mMia;
    private final int mMaxContCurrent;
    private final int mMaxPeakCurrent;
    private final int mPeakCurrentDuration;
    private final FeedbackDevice mEncoder;
    private final double mDeadband;

    /**
     * Creates a new MotorConfig object to store motor controller configuration settings
     * @param p The propritional constant
     * @param i The integral constant
     * @param d The derivitive constant
     * @param f The feed-forward constant
     * @param mia The maximum integral accumulator constant
     * @param maxCurrent The maximum current the motor will draw continuously
     * @param maxPeakCurrent The maximum current the motor will draw at peak
     * @param peakCurrentDuration The duration of the peak current(in ms)
     * @param encoder The type of encoder the motor uses
     * @param deadband The deadband(0.0-1.0) to be applied to the motor controller
     */
    public MotorConfig(double p, double i, double d, double f, double mia, int maxContCurrent, int maxPeakCurrent, int peakCurrentDuration, FeedbackDevice encoder, double deadband) {
        mP = p;
        mI = i;
        mD = d;
        mF = f;
        mMia = mia;
        mMaxContCurrent = maxContCurrent;
        mMaxPeakCurrent = maxPeakCurrent;
        mPeakCurrentDuration = peakCurrentDuration;
        mEncoder = encoder;
        mDeadband = deadband;
    }

    /**
     * @return The proportional constant
     */
    public double getP() {
        return mP;
    }

    /**
     * @return The integral constant
     */
    public double getI() {
        return mI;
    }

    /**
     * @return The derivitive constant
     */
    public double getD() {
        return mD;
    }

    /**
     * @return The feed-forward constant
     */
    public double getF() {
        return mF;
    }

    /**
     * @return The max integral accumulator constant
     */
    public double getMIA() {
        return mMia;
    }

    /**
     * @return The max current the motor will draw continuously
     */
    public int getContinuousCurrent() {
        return mMaxContCurrent;
    }

    /**
     * @return The max current the motor will draw at peak
     */
    public int getPeakCurrent() {
        return mMaxPeakCurrent;
    }

    /**
     * @return The maximum current the motor will draw at peak
     */
    public int getPeakDuration() {
        return mPeakCurrentDuration;
    }

    /**
     * @return The encoder type
     */
    public FeedbackDevice getEncoder() {
        return mEncoder;
    }

    /**
     * @return The size of the deadband
     */
    public double getDeadband() {
        return mDeadband;
    }
    
    
    /**
     * Configure a Talon SRX with the specified MotorConfig
     * @param talon The Talon SRX to be configure
     * @param config The configuration to apply
     * @param slot The PID slot to use
     */
    public static void configTalon(TalonSRX talon, MotorConfig config, int slot) {
        talon.config_kP(slot, config.getP());
        talon.config_kI(slot, config.getI());
        talon.config_kD(slot, config.getD());
        talon.config_kF(slot, config.getF());
        talon.configMaxIntegralAccumulator(slot, config.getMIA());
        talon.configContinuousCurrentLimit(config.getContinuousCurrent());
        talon.configPeakCurrentLimit(config.getPeakCurrent());
        talon.configPeakCurrentDuration(config.getPeakDuration());
        talon.enableCurrentLimit(true);
        talon.configSelectedFeedbackSensor(config.getEncoder());
        talon.configNeutralDeadband(config.getDeadband());
    }
    
    /**
     * Configure a Talon FX with the specified MotorConfig
     * @param talon The Talon FX to be configure
     * @param config The MotorConfig to apply
     * @param slot The PID slot to use
     */
    public static void configTalon(TalonFX talon, MotorConfig config, int slot) {
        talon.config_kP(slot, config.getP());
        talon.config_kI(slot, config.getI());
        talon.config_kD(slot, config.getD());
        talon.config_kF(slot, config.getF());
        talon.configMaxIntegralAccumulator(slot, config.getMIA());
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, config.getContinuousCurrent(), config.getPeakCurrent(), config.getPeakDuration()));
        talon.configSelectedFeedbackSensor(config.getEncoder());
        talon.configNeutralDeadband(config.getDeadband());
    }

    /**
     * Resets a Talon SRX's constants
     * @param talon The Talon SRX to be reset
     */
    public static void resetTalon(TalonSRX talon) {
        talon.configFactoryDefault();
    }

    /**
     * Resets a Talon FX's constants
     * @param talon The Talon FX to be reset
     */
    public static void resetTalon(TalonFX talon) {
        talon.configFactoryDefault();
    }
}